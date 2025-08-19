using System;
using MathNet.Numerics.LinearAlgebra;
using UniRx;

namespace TrajectoryPlanning.Robot
{
    public interface IReadOnlyRobotModel
    {
        string Id { get; }
        int Dof { get; }
        string[] JointNames { get; }
        float[] JointMinLimits { get; }
        float[] JointMaxLimits { get; }
        Vector<float> InitialState { get; }
        IReadOnlyReactiveProperty<float[]> JointsMaxVelocity { get; }
        IReadOnlyReactiveProperty<float[]> JointsMaxAcceleration { get; }
        IReadOnlyReactiveProperty<Vector<float>> CurrentState { get; }
    }

    public interface IRobotModel : IReadOnlyRobotModel
    {
        void SetState(Vector<float> state);
        void SetVelocityLimits(float[] limits);
        void SetAccelerationLimits(float[] limits);
    }

    public class RobotModel : IRobotModel, IDisposable
    {
        private readonly ReactiveProperty<Vector<float>> _currentState;
        private readonly ReactiveProperty<float[]> _jointMaxAcceleration;
        private readonly ReactiveProperty<float[]> _jointMaxVelocity;

        public IReadOnlyReactiveProperty<Vector<float>> CurrentState => _currentState;
        public Vector<float> InitialState { get; }
        public int Dof { get; }
        public string Id { get; }
        public string[] JointNames { get; }
        public float[] JointMinLimits { get; }
        public float[] JointMaxLimits { get; }
        public IReadOnlyReactiveProperty<float[]> JointsMaxVelocity => _jointMaxVelocity;
        public IReadOnlyReactiveProperty<float[]> JointsMaxAcceleration => _jointMaxAcceleration;

        public RobotModel(RobotModelDto config)
        {
            Dof = config.dof;
            Id = config.model;
            JointNames = config.jointNames;
            JointMinLimits = config.jointMinRadians;
            JointMaxLimits = config.jointMaxRadians;
            _jointMaxVelocity = new ReactiveProperty<float[]>(config.jointMaxVelocityRadPerSec);
            _jointMaxAcceleration = new ReactiveProperty<float[]>(
                config.jointMaxAccelerationRadPerSec2
            );
            InitialState = Vector<float>.Build.Dense(config.initialStateRadians);
            _currentState = new ReactiveProperty<Vector<float>>(InitialState);
        }

        public void SetState(Vector<float> state)
        {
            _currentState.Value = state;
        }

        public void SetVelocityLimits(float[] limits)
        {
            _jointMaxVelocity.Value = limits;
        }

        public void SetAccelerationLimits(float[] limits)
        {
            _jointMaxAcceleration.Value = limits;
        }

        public void Dispose()
        {
            _currentState.Dispose();
            _jointMaxVelocity.Dispose();
            _jointMaxAcceleration.Dispose();
        }
    }
}
