using System;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using TrajectoryPlanning.Robot;
using UniRx;
using UnityEngine;
using UnityEngine.UIElements;
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
            _view.BuildLimitEditors(_robotModel.JointNames, _robotModel.Dof);
            _robotModel.CurrentState.Subscribe(UpdateState).AddTo(_disposables);
            _robotModel
                .JointsMaxVelocity.Subscribe(v => _view.SetVelocityLimitsDegPerSec(v))
                .AddTo(_disposables);
            _robotModel
                .JointsMaxAcceleration.Subscribe(v => _view.SetAccelerationLimitsDegPerSec2(v))
                .AddTo(_disposables);

            _view.SetVelocityLimitsDegPerSec(_robotModel.JointsMaxVelocity.Value);
            _view.SetAccelerationLimitsDegPerSec2(_robotModel.JointsMaxAcceleration.Value);

            _view.ForEachVelocityLimitField(
                (i, field) =>
                {
                    EventCallback<ChangeEvent<float>> handler = evt =>
                    {
                        var current = _robotModel.JointsMaxVelocity.Value.ToArray();
                        current[i] = evt.newValue * Mathf.Deg2Rad;
                        _robotModel.SetVelocityLimits(current);
                    };
                    field.RegisterValueChangedCallback(handler);
                    _disposables.Add(
                        Disposable.Create(() => field.UnregisterValueChangedCallback(handler))
                    );
                }
            );

            _view.ForEachAccelerationLimitField(
                (i, field) =>
                {
                    EventCallback<ChangeEvent<float>> handler = evt =>
                    {
                        var current = _robotModel.JointsMaxAcceleration.Value.ToArray();
                        current[i] = evt.newValue * Mathf.Deg2Rad;
                        _robotModel.SetAccelerationLimits(current);
                    };
                    field.RegisterValueChangedCallback(handler);
                    _disposables.Add(
                        Disposable.Create(() => field.UnregisterValueChangedCallback(handler))
                    );
                }
            );
            _view
                .MoveButton.OnClickAsObservable()
                .Subscribe(_ => MoveToTargets())
                .AddTo(_disposables);
            _view
                .StopButton.OnClickAsObservable()
                .Subscribe(_ => _controller.Stop())
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
