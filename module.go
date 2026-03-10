package hellomotion

import (
	"context"
	"errors"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
)

var (
	Hellomotion      = resource.NewModel("viamdemo", "HelloMotion", "hellomotion")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(generic.API, Hellomotion,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newHellomotionHellomotion,
		},
	)
}

type Config struct {
	Arm          string `json:"arm"`
	Gripper      string `json:"gripper"`
	PickupVision string `json:"pickup_vision"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {

	deps := []string{motion.Named("builtin").String()}

	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	deps = append(deps, cfg.Arm)

	if cfg.Gripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "gripper")
	}
	deps = append(deps, cfg.Gripper)

	if cfg.PickupVision == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "pickup_vision")
	}
	deps = append(deps, cfg.PickupVision)

	return deps, nil, nil
}

type hellomotionHellomotion struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	motion       motion.Service
	arm          arm.Arm
	gripper      gripper.Gripper
	pickupVision vision.Service

	cancelCtx  context.Context
	cancelFunc func()
}

func newHellomotionHellomotion(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewHellomotion(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewHellomotion(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {
	var err error
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &hellomotionHellomotion{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}

	s.motion, err = motion.FromProvider(deps, "builtin")
	if err != nil {
		return nil, err
	}

	s.arm, err = arm.FromProvider(deps, conf.Arm)
	if err != nil {
		return nil, err
	}

	s.gripper, err = gripper.FromProvider(deps, conf.Gripper)
	if err != nil {
		return nil, err
	}

	s.pickupVision, err = vision.FromProvider(deps, conf.PickupVision)
	if err != nil {
		return nil, err
	}

	return s, nil
}

func (s *hellomotionHellomotion) Name() resource.Name {
	return s.name
}

func (s *hellomotionHellomotion) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if wave, ok := cmd["wave"].(bool); ok && wave {
		s.logger.Info("you asked me to wave")
		s.handleWave(ctx)
		return cmd, nil
	}

	if pickup, ok := cmd["pickup"].(bool); ok && pickup {
		s.logger.Info("you asked me to pickup")
		pose, err := s.handlePickup(ctx)
		if err != nil {
			return nil, err
		}
		pt := pose.Point()
		return map[string]interface{}{
			"pickup": true,
			"pose": map[string]interface{}{
				"x": pt.X,
				"y": pt.Y,
				"z": pt.Z,
			},
		}, nil
	}

	return nil, fmt.Errorf("unknown command: expected 'wave' or 'pickup'")
}

func (s *hellomotionHellomotion) handleWave(ctx context.Context) {
	currentPose, err := s.motion.GetPose(ctx, s.cfg.Arm, "world", nil, nil)
	if err != nil {
		s.logger.Errorf("could not get current pose: %s ", err)
		return
	}
	s.logger.Infof("got arm pose: %+v", currentPose)

	destinationPose := spatialmath.NewPose(currentPose.Pose().Point().Add(r3.Vector{X: 0, Y: 20, Z: 0}), currentPose.Pose().Orientation())

	destinationPoseInWorld := referenceframe.NewPoseInFrame("world", destinationPose)

	s.motion.Move(ctx, motion.MoveReq{ComponentName: s.cfg.Arm, Destination: destinationPoseInWorld})

}

func (s *hellomotionHellomotion) handlePickup(ctx context.Context) (spatialmath.Pose, error) {
	objects, err := s.pickupVision.GetObjectPointClouds(ctx, "", nil)
	if err != nil {
		return nil, fmt.Errorf("could not get object point clouds: %w", err)
	}
	if len(objects) == 0 {
		return nil, errors.New("no objects detected")
	}

	biggest := objects[0]
	for _, obj := range objects[1:] {
		if obj.Size() > biggest.Size() {
			biggest = obj
		}
	}

	if biggest.Geometry == nil {
		return nil, errors.New("biggest object has no geometry")
	}

	objPose := biggest.Geometry.Pose()
	s.logger.Infof("biggest object has %d points at pose: %+v (label: %s)", biggest.Size(), objPose, biggest.Geometry.Label())

	// Move arm to object pose, offset +400mm in Z
	abovePose := spatialmath.NewPose(
		objPose.Point().Add(r3.Vector{X: 0, Y: 0, Z: 400}),
		objPose.Orientation(),
	)
	destination := referenceframe.NewPoseInFrame("world", abovePose)
	if _, err := s.motion.Move(ctx, motion.MoveReq{ComponentName: s.cfg.Arm, Destination: destination}); err != nil {
		return nil, fmt.Errorf("could not move arm above object: %w", err)
	}
	s.logger.Info("arm moved above object, grabbing")

	// Grab with gripper
	if grabbed, err := s.gripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("gripper grab failed: %w", err)
	} else if !grabbed {
		s.logger.Warn("gripper closed but did not detect an object")
	}

	return objPose, nil
}

func (s *hellomotionHellomotion) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
