package hellomotion

import (
	"context"
	"errors"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	generic "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
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
	Arm     string `json:"arm"`
	Gripper string `json:"gripper"`
	Camera  string `json:"camera"`
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

	if cfg.Camera == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera")
	}
	deps = append(deps, cfg.Camera)

	return deps, nil, nil
}

type hellomotionHellomotion struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	motion   motion.Service
	arm      arm.Arm
	gripper  gripper.Gripper
	camera   camera.Camera
	frameSys framesystem.Service

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

	s.camera, err = camera.FromProvider(deps, conf.Camera)
	if err != nil {
		return nil, err
	}

	s.frameSys, err = framesystem.FromDependencies(deps)
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
	// Get point cloud from camera
	pc, err := s.camera.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("could not get point cloud from camera: %w", err)
	}
	if pc.Size() == 0 {
		return nil, errors.New("empty point cloud from camera")
	}
	s.logger.Infof("got point cloud with %d points from camera", pc.Size())

	// Transform point cloud from camera frame to world frame
	worldPC, err := s.frameSys.TransformPointCloud(ctx, pc, s.cfg.Camera, referenceframe.World)
	if err != nil {
		return nil, fmt.Errorf("could not transform point cloud to world frame: %w", err)
	}
	s.logger.Infof("transformed point cloud to world frame: %d points", worldPC.Size())

	// Compute bounding box to get the center pose of the point cloud
	bbox, err := pointcloud.BoundingBoxFromPointCloud(worldPC)
	if err != nil {
		return nil, fmt.Errorf("could not compute bounding box: %w", err)
	}

	objPose := bbox.Pose()
	s.logger.Infof("bounding box center at pose: %+v", objPose)

	// Move arm to object pose, offset +170mm in Z
	abovePose := spatialmath.NewPose(
		objPose.Point().Add(r3.Vector{X: 0, Y: 0, Z: 170}),
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: 0},
	)
	s.logger.Infof("object pose: %+v", objPose.Point())
	s.logger.Infof("above pose (+170mm Z): %+v", abovePose.Point())
	destination := referenceframe.NewPoseInFrame("world", abovePose)
	if _, err := s.motion.Move(ctx, motion.MoveReq{ComponentName: s.cfg.Arm, Destination: destination}); err != nil {
		return nil, fmt.Errorf("could not move arm above object to %+v: %w", abovePose.Point(), err)
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
