using MathNet.Numerics.LinearAlgebra;

namespace TrajectoryPlanning.TrajectoryPlanner
{
    public record Trajectory(
        Vector<float>[] Positions,
        Vector<float>[] Velocity,
        Vector<float>[] Acceleration,
        float[] TimeStamps
    );
}
