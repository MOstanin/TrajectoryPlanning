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
            const float rate = 100f;
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
            const float rate = 100f;
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

        [Test]
        public void WhenPlanMoveJ_AndTwoJoints_AndSlowAndFast_AndSmallDistance_ThenBothTriangular()
        {
            // Arrange
            const float precision = 1e-4f;
            const float rate = 100f;
            var vLimits = new[] { 0.5f, 0.5f };
            var aLimits = new[] { 0.5f, 5f };
            var q0 = Vector<float>.Build.Dense(new[] { 0f, 0f });
            var qTarget = Vector<float>.Build.Dense(new[] { 0.1f, 0.1f });

            var model = Substitute.For<IReadOnlyRobotModel>();
            model.Id.Returns("MockRobot2J");
            model.Dof.Returns(2);
            model.JointsMaxVelocity.Returns(new ReactiveProperty<float[]>(vLimits));
            model.JointsMaxAcceleration.Returns(new ReactiveProperty<float[]>(aLimits));

            var planner = new TrapezoidalTrajectoryPlanner("twoj-tri", rate);

            // Act
            var traj = planner.PlanMoveJ(model, q0, qTarget);

            // Assert
            traj.Positions.Length.Should().Be(traj.TimeStamps.Length);
            traj.Positions.First()[0].Should().BeApproximately(q0[0], precision);
            traj.Positions.First()[1].Should().BeApproximately(q0[1], precision);
            traj.Positions.Last()[0].Should().BeApproximately(qTarget[0], precision);
            traj.Positions.Last()[1].Should().BeApproximately(qTarget[1], precision);
            traj.Velocity.Last()[0].Should().BeApproximately(0f, precision);
            traj.Velocity.Last()[1].Should().BeApproximately(0f, precision);

            var cruiseCountJoint0 = traj.Acceleration.Zip(
                traj.Velocity,
                (a, v) => (a: Math.Abs(a[0]), v: Math.Abs(v[0]))
            )
                .Count(x => x is { a: <= precision, v: > precision });
            var cruiseCountJoint1 = traj.Acceleration.Zip(
                traj.Velocity,
                (a, v) => (a: Math.Abs(a[1]), v: Math.Abs(v[1]))
            )
                .Count(x => x is { a: <= precision, v: > precision });
            cruiseCountJoint0.Should().Be(0);
            cruiseCountJoint1.Should().Be(0);
        }

        [Test]
        public void WhenPlanMoveJ_AndTwoJoints_AndSlowAndFast_AndLargeDistance_BothTrapezoidal()
        {
            // Arrange
            const float precision = 1e-4f;
            const float rate = 100f;
            var vLimits = new[] { 0.5f, 0.5f };
            var aLimits = new[] { 0.5f, 5f };
            var q0 = Vector<float>.Build.Dense(new[] { 0f, 0f });
            var qTarget = Vector<float>.Build.Dense(new[] { 1f, 1f });

            var model = Substitute.For<IReadOnlyRobotModel>();
            model.Id.Returns("MockRobot2J");
            model.Dof.Returns(2);
            model.JointsMaxVelocity.Returns(new ReactiveProperty<float[]>(vLimits));
            model.JointsMaxAcceleration.Returns(new ReactiveProperty<float[]>(aLimits));

            var planner = new TrapezoidalTrajectoryPlanner("twoj-trap", rate);

            // Act
            var traj = planner.PlanMoveJ(model, q0, qTarget);

            // Assert
            traj.Positions.Length.Should().Be(traj.TimeStamps.Length);
            traj.Positions.First()[0].Should().BeApproximately(q0[0], precision);
            traj.Positions.First()[1].Should().BeApproximately(q0[1], precision);
            traj.Positions.Last()[0].Should().BeApproximately(qTarget[0], precision);
            traj.Positions.Last()[1].Should().BeApproximately(qTarget[1], precision);
            traj.Velocity.Last()[0].Should().BeApproximately(0f, precision);
            traj.Velocity.Last()[1].Should().BeApproximately(0f, precision);

            var cruiseCountJoint0 = traj.Acceleration.Zip(
                traj.Velocity,
                (a, v) => (a: Math.Abs(a[0]), v: Math.Abs(v[0]))
            )
                .Count(x => x is { a: <= precision, v: > precision });
            var cruiseCountJoint1 = traj.Acceleration.Zip(
                traj.Velocity,
                (a, v) => (a: Math.Abs(a[1]), v: Math.Abs(v[1]))
            )
                .Count(x => x is { a: <= precision, v: > precision });
            cruiseCountJoint0.Should().BeGreaterThan(0);
            cruiseCountJoint1.Should().BeGreaterThan(0);
        }
    }
}
