using System.Linq;
using FluentAssertions;
using MathNet.Numerics.LinearAlgebra;
using NUnit.Framework;
using TrajectoryPlanning.Robot;
using UniRx;

namespace TrajectoryPlanning.Tests
{
    [TestFixture]
    public class RobotModelTests
    {
        private static RobotModelDto CreateDto()
        {
            return new RobotModelDto(
                model: "TestRobot",
                dof: 2,
                jointNames: new[] { "J0", "J1" },
                jointMinRadians: new[] { -1f, -2f },
                jointMaxRadians: new[] { 1f, 2f },
                jointMaxVelocityRadPerSec: new[] { 0.5f, 1f },
                jointMaxAccelerationRadPerSec2: new[] { 1f, 2f },
                initialStateRadians: new[] { 0.1f, 0.2f }
            );
        }

        [Test]
        public void WhenConstructed_ThenPropertiesInitializedFromDto()
        {
            // Arrange
            var dto = CreateDto();

            // Act
            using var model = new RobotModel(dto);

            // Assert
            model.Id.Should().Be(dto.model);
            model.Dof.Should().Be(dto.dof);
            model.JointNames.Should().Equal(dto.jointNames);
            model.JointMinLimits.Should().Equal(dto.jointMinRadians);
            model.JointMaxLimits.Should().Equal(dto.jointMaxRadians);
            model.JointsMaxVelocity.Value.Should().Equal(dto.jointMaxVelocityRadPerSec);
            model.JointsMaxAcceleration.Value.Should().Equal(dto.jointMaxAccelerationRadPerSec2);
            model.InitialState.ToArray().Should().Equal(dto.initialStateRadians);
            model.CurrentState.Value.ToArray().Should().Equal(dto.initialStateRadians);
        }

        [Test]
        public void WhenSetState_ThenCurrentStateUpdatesAndNotifies()
        {
            // Arrange
            var dto = CreateDto();
            using var model = new RobotModel(dto);
            var observed = Vector<float>.Build.Dense(dto.initialStateRadians);
            var notifications = 0;
            using var sub = model.CurrentState.Subscribe(v =>
            {
                observed = v;
                notifications++;
            });
            var next = Vector<float>.Build.Dense(new[] { 0.3f, 0.4f });

            // Act
            model.SetState(next);

            // Assert
            notifications.Should().BeGreaterThan(0);
            observed.ToArray().Should().Equal(next.ToArray());
            model.CurrentState.Value.ToArray().Should().Equal(next.ToArray());
        }

        [Test]
        public void WhenSetVelocityLimits_ThenReactivePropertyUpdatesAndNotifies()
        {
            // Arrange
            var dto = CreateDto();
            using var model = new RobotModel(dto);
            var last = model.JointsMaxVelocity.Value.ToArray();
            var notifications = 0;
            using var sub = model.JointsMaxVelocity.Subscribe(v =>
            {
                last = v;
                notifications++;
            });
            var newLimits = new[] { 0.7f, 1.2f };

            // Act
            model.SetVelocityLimits(newLimits);

            // Assert
            notifications.Should().BeGreaterThan(0);
            last.Should().Equal(newLimits);
            model.JointsMaxVelocity.Value.Should().Equal(newLimits);
        }

        [Test]
        public void WhenSetAccelerationLimits_ThenReactivePropertyUpdatesAndNotifies()
        {
            // Arrange
            var dto = CreateDto();
            using var model = new RobotModel(dto);
            var last = model.JointsMaxAcceleration.Value.ToArray();
            var notifications = 0;
            using var sub = model.JointsMaxAcceleration.Subscribe(v =>
            {
                last = v;
                notifications++;
            });
            var newLimits = new[] { 1.5f, 2.5f };

            // Act
            model.SetAccelerationLimits(newLimits);

            // Assert
            notifications.Should().BeGreaterThan(0);
            last.Should().Equal(newLimits);
            model.JointsMaxAcceleration.Value.Should().Equal(newLimits);
        }
    }
}
