using System;
using TrajectoryPlanning.Services;
using UniRx;
using Zenject;

namespace TrajectoryPlanning.Robot
{
    public sealed class RobotSettingAdapter : IInitializable, IDisposable
    {
        private readonly ISettingSavingService<string, RobotModelDto> _service;
        private readonly IReadOnlyRobotModel _robotModel;
        private readonly CompositeDisposable _disposables = new();

        public RobotSettingAdapter(
            ISettingSavingService<string, RobotModelDto> service,
            IReadOnlyRobotModel robotModel
        )
        {
            _robotModel = robotModel;
            _service = service;
        }

        public void Initialize()
        {
            Observable
                .Merge(_robotModel.JointsMaxVelocity, _robotModel.JointsMaxAcceleration)
                .Subscribe(_ => Save())
                .AddTo(_disposables);
        }

        private void Save()
        {
            var dto = new RobotModelDto(
                _robotModel.Id,
                _robotModel.Dof,
                _robotModel.JointNames,
                _robotModel.JointMinLimits,
                _robotModel.JointMaxLimits,
                _robotModel.JointsMaxVelocity.Value,
                _robotModel.JointsMaxAcceleration.Value,
                _robotModel.InitialState
            );
            _service.Save(dto);
        }

        public void Dispose()
        {
            _disposables.Dispose();
        }
    }
}
