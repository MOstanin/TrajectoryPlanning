using System;
using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanning.Robot;
using UniRx;
using Zenject;

namespace TrajectoryPlanning.UI
{
    public class RobotUIPresenter : IInitializable, IDisposable
    {
        private readonly IRobotModel _robotModel;
        private readonly IRobotController _controller;
        private readonly RobotView _view;
        private readonly CompositeDisposable _disposables = new();

        public RobotUIPresenter(IRobotModel robotModel, RobotView view, IRobotController controller)
        {
            _robotModel = robotModel;
            _view = view;
            _controller = controller;
        }

        public void Initialize()
        {
            _view.SetModel(_robotModel.Id, _robotModel.Dof);
            _view.BuildJoints(_robotModel.JointNames, _robotModel.Dof);
            _robotModel.CurrentState.Subscribe(UpdateState).AddTo(_disposables);
            _view
                .MoveButton.OnClickAsObservable()
                .Subscribe(_ => MoveToTargets())
                .AddTo(_disposables);
            UpdateState(_robotModel.InitialState);
        }

        private void UpdateState(Vector<float> state)
        {
            _view.UpdateState(state);
        }

        private void MoveToTargets()
        {
            var target = _view.GetTargetStateRadians(_robotModel.Dof);
            _controller.MoveTo(target);
        }

        public void Dispose()
        {
            _disposables.Dispose();
        }
    }
}
