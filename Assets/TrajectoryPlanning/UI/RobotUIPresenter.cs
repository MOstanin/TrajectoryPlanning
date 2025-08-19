using System;
using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanning.Robot;
using UniRx;
using Zenject;

namespace TrajectoryPlanning.UI
{
    public class RobotUIPresenter : IInitializable, IDisposable
    {
        private readonly IReadOnlyRobotModel _robotModel;
        private readonly RobotView _view;
        private readonly CompositeDisposable _disposables = new();

        public RobotUIPresenter(IReadOnlyRobotModel robotModel, RobotView view)
        {
            _robotModel = robotModel;
            _view = view;
        }

        public void Initialize()
        {
            _view.SetModel(_robotModel.Id, _robotModel.Dof);
            _view.BuildJoints(_robotModel.JointNames, _robotModel.Dof);
            _robotModel.CurrentState.Subscribe(UpdateState).AddTo(_disposables);
            UpdateState(_robotModel.InitialState);
        }

        private void UpdateState(Vector<float> state)
        {
            _view.UpdateState(state);
        }

        public void Dispose()
        {
            _disposables.Dispose();
        }
    }
}
