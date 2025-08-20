using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanning.Robot;
using UniRx;

namespace TrajectoryPlanning.TrajectoryPlanner
{
    public interface ITrajectoryPlanner
    {
        string Id { get; }
        IReactiveProperty<float> Rate { get; }
        Trajectory PlanMoveJ(IReadOnlyRobotModel model, Vector<float> q0, Vector<float> qTarget);
    }
}
