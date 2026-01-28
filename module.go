package hellomotion

import (
	"context"
	"errors"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	// "go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
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
	Arm string `json:"arm"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {

	deps := []string{motion.Named("builtin").String()}

	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	deps = append(deps, cfg.Arm)

	return deps, nil, nil
}

type hellomotionHellomotion struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	motion motion.Service
	arm    arm.Arm

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

	return s, nil
}

func (s *hellomotionHellomotion) Name() resource.Name {
	return s.name
}

func (s *hellomotionHellomotion) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	command, ok := cmd["wave"].(bool)
	if !ok {
		return nil, fmt.Errorf("missing or invalid 'command' field")
	}

	if command == true {
		s.logger.Info("you asked me to wave")
		s.handleWave(ctx)
		return cmd, nil
	}

	return nil, fmt.Errorf("unknown command")
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

func (s *hellomotionHellomotion) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
