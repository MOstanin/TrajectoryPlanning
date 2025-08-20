using System;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanning.TrajectoryPlanner;
using UniRx;

namespace TrajectoryPlanning.Robot
{
    public interface IRobotController
    {
        void MoveTo(Vector<float> target);
        void Stop();
    }

    public class RobotController : IRobotController, IDisposable
    {
        private readonly IRobotModel _robotModel;
        private readonly ITrajectoryPlanner _planner;
        private IDisposable _moveSimulation = Disposable.Empty;

        public RobotController(IRobotModel robotModel, ITrajectoryPlanner planner)
        {
            _robotModel = robotModel;
            _planner = planner;
        }

        public void MoveTo(Vector<float> target)
        {
            _moveSimulation.Dispose();

            var dof = _robotModel.Dof;
            var clamped = Vector<float>.Build.Dense(dof);
            for (var i = 0; i < dof; i++)
            {
                var min = _robotModel.JointMinLimits[i];
                var max = _robotModel.JointMaxLimits[i];
                var value = target[i];
                if (value < min)
                    value = min;
                if (value > max)
                    value = max;
                clamped[i] = value;
            }

            var q0 = _robotModel.CurrentState.Value;
            var trajectory = _planner.PlanMoveJ(_robotModel, q0, clamped);
            var steps = trajectory.Positions.Length;
            if (steps == 0)
            {
                _robotModel.SetState(clamped);
                return;
            }

            var intervalSec = 1.0 / _planner.Rate.Value;

            _moveSimulation = Observable
                .Interval(TimeSpan.FromSeconds(intervalSec), Scheduler.MainThread)
                .Take(steps)
                .Subscribe(
                    idx => _robotModel.SetState(trajectory.Positions[(int)idx]),
                    () => _robotModel.SetState(trajectory.Positions.Last())
                );
        }

        public void Stop()
        {
            _moveSimulation.Dispose();
        }

        public void Dispose()
        {
            _moveSimulation.Dispose();
        }
    }
}
