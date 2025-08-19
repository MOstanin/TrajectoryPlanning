using System;
using MathNet.Numerics.LinearAlgebra;
using UniRx;

namespace TrajectoryPlanning.Robot
{
    public interface IReadOnlyRobotModel
    {
        IReadOnlyReactiveProperty<Vector<float>> CurrentState { get; }
        int Dof { get; }
        IReadOnlyReactiveProperty<float[]> JointMaxVelocityRadPerSec { get; }
        IReadOnlyReactiveProperty<float[]> JointMaxAccelerationRadPerSec2 { get; }
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
        public IReadOnlyReactiveProperty<Vector<float>> CurrentState => _currentState;
        public int Dof { get; }
        private readonly ReactiveProperty<float[]> _jointMaxVelocity;
        private readonly ReactiveProperty<float[]> _jointMaxAcceleration;
        public IReadOnlyReactiveProperty<float[]> JointMaxVelocityRadPerSec => _jointMaxVelocity;
        public IReadOnlyReactiveProperty<float[]> JointMaxAccelerationRadPerSec2 =>
            _jointMaxAcceleration;

        public RobotModel(RobotModelDto config, float[] initialState)
        {
            Dof = config.dof;
            _jointMaxVelocity = new ReactiveProperty<float[]>(config.jointMaxVelocityRadPerSec);
            _jointMaxAcceleration = new ReactiveProperty<float[]>(
                config.jointMaxAccelerationRadPerSec2
            );
            _currentState = new ReactiveProperty<Vector<float>>(
                Vector<float>.Build.Dense(initialState)
            );
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
