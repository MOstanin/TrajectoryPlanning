using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanning.Robot;

namespace TrajectoryPlanning.TrajectoryPlanner
{
    public interface ITrajectoryPlanner
    {
        string Id { get; }
        float Rate { get; }
        Trajectory PlanMoveJ(IReadOnlyRobotModel model, Vector<float> q0, Vector<float> qTarget);
    }
}
