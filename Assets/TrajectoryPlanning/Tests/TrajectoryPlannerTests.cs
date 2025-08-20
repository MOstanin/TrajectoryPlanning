using System;
using System.Linq;
using FluentAssertions;
using MathNet.Numerics.LinearAlgebra;
using NSubstitute;
using NUnit.Framework;
using TrajectoryPlanning.Planner;
using TrajectoryPlanning.Robot;
using UniRx;

namespace TrajectoryPlanning.Tests
{
    [TestFixture]
    public class TrajectoryPlannerTests
    {
        [Test]
        public void WhenPlanMoveJ_AndTriangularProfile_ThenVelocityLowerLimit()
        {
            // Arrange
            const float precision = 1e-4f;
            const float rate = 10f;
            const float vLim = 2f;
            const float aLim = 1f;
            var q0 = Vector<float>.Build.Dense(new[] { 0f });
            var qTarget = Vector<float>.Build.Dense(new[] { 1f });

            var model = Substitute.For<IReadOnlyRobotModel>();
            model.Id.Returns("MockRobot");
            model.Dof.Returns(1);
            model.JointsMaxVelocity.Returns(new ReactiveProperty<float[]>(new[] { vLim }));
            model.JointsMaxAcceleration.Returns(new ReactiveProperty<float[]>(new[] { aLim }));

            var planner = new TrapezoidalTrajectoryPlanner("tri-test", rate);

            // Act
            var traj = planner.PlanMoveJ(model, q0, qTarget);

            // Assert
            traj.Positions.Length.Should().Be(traj.TimeStamps.Length);

            traj.Positions.First()[0].Should().BeApproximately(q0[0], precision);
            traj.Positions.Last()[0].Should().BeApproximately(qTarget[0], precision);
            traj.Velocity.Last()[0].Should().BeApproximately(0f, precision);

            var zeroAccNonZeroVel = traj.Acceleration.Zip(
                traj.Velocity,
                (a, v) => (a: Math.Abs(a[0]), v: Math.Abs(v[0]))
            )
                .Count(x => x is { a: <= precision, v: > precision });
            zeroAccNonZeroVel.Should().Be(0);

            var maxVel = traj.Velocity.Max(v => v[0]);
            maxVel.Should().BeLessThan(vLim);
        }

        [Test]
        public void WhenPlanMoveJ_AndTrapezoidalProfile_ThenMaxVelocityIsOnLimit()
        {
            // Arrange
            const float precision = 1e-4f;
            const float rate = 10f;
            const float vLim = 2f;
            const float aLim = 1f;
            var q0 = Vector<float>.Build.Dense(new[] { 0f });
            var qTarget = Vector<float>.Build.Dense(new[] { 10f });

            var model = Substitute.For<IReadOnlyRobotModel>();
            model.Id.Returns("MockRobot");
            model.Dof.Returns(1);
            model.JointsMaxVelocity.Returns(new ReactiveProperty<float[]>(new[] { vLim }));
            model.JointsMaxAcceleration.Returns(new ReactiveProperty<float[]>(new[] { aLim }));

            var planner = new TrapezoidalTrajectoryPlanner("trap-test", rate);

            // Act
            var traj = planner.PlanMoveJ(model, q0, qTarget);

            // Assert
            traj.Positions.Length.Should().Be(traj.TimeStamps.Length);

            traj.Positions.First()[0].Should().BeApproximately(q0[0], precision);
            traj.Positions.Last()[0].Should().BeApproximately(qTarget[0], precision);
            traj.Velocity.Last()[0].Should().BeApproximately(0f, precision);

            var zeroAccNonZeroVel = traj.Acceleration.Zip(
                traj.Velocity,
                (a, v) => (a: Math.Abs(a[0]), v: Math.Abs(v[0]))
            )
                .Count(x => x is { a: <= precision, v: > precision });
            zeroAccNonZeroVel.Should().BeGreaterThan(0);

            var maxVel = traj.Velocity.Max(v => v[0]);
            maxVel.Should().BeApproximately(vLim, precision);
        }
    }
}
