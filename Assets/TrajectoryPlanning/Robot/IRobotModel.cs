using System;
using MathNet.Numerics.LinearAlgebra;
using UniRx;

namespace TrajectoryPlanning.Robot
{
    public interface IReadOnlyRobotModel
    {
        IReadOnlyReactiveProperty<Vector<float>> CurrentState { get; }
    }

    public interface IRobotModel : IReadOnlyRobotModel
    {
        void SetState(Vector<float> state);
    }

    public class RobotModel : IRobotModel, IDisposable
    {
        private readonly ReactiveProperty<Vector<float>> _currentState;
        public IReadOnlyReactiveProperty<Vector<float>> CurrentState => _currentState;

        public RobotModel(float[] initialState)
        {
            _currentState = new ReactiveProperty<Vector<float>>(
                Vector<float>.Build.Dense(initialState)
            );
        }

        public void SetState(Vector<float> state)
        {
            _currentState.Value = state;
        }

        public void Dispose()
        {
            _currentState.Dispose();
        }
    }
}
