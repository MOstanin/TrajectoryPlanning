using System;
using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanning.Robot;
using UniRx;
using UnityEngine;
using Zenject;

namespace TrajectoryPlanning.UI
{
    public class RobotUIPresenter : IInitializable, IDisposable
    {
        private readonly IRobotModel _robotModel;
        private readonly RobotView _view;
        private readonly CompositeDisposable _disposables = new();

        public RobotUIPresenter(IRobotModel robotModel, RobotView view)
        {
            _robotModel = robotModel;
            _view = view;
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
            for (var i = 0; i < _robotModel.Dof; i++)
            {
                var min = _robotModel.JointMinLimits[i];
                var max = _robotModel.JointMaxLimits[i];
                target[i] = Mathf.Clamp(target[i], min, max);
            }
            _robotModel.SetState(target);
        }

        public void Dispose()
        {
            _disposables.Dispose();
        }
    }
}
