using System;
using System.IO;
using Newtonsoft.Json;
using TrajectoryPlanning.Services;
using UniRx;
using UnityEngine;
using Zenject;

namespace TrajectoryPlanning.Robot
{
    public sealed class RobotSettingAdapter : IInitializable, IDisposable
    {
        private readonly ISettingSavingService<RobotModelDto> _service;
        private readonly IReadOnlyRobotModel _robotModel;
        private readonly CompositeDisposable _disposables = new();

        public RobotSettingAdapter(
            ISettingSavingService<RobotModelDto> service,
            IReadOnlyRobotModel robotModel
        )
        {
            _robotModel = robotModel;
            _service = service;
        }

        public void Initialize()
        {
            Observable
                .Merge(
                    _robotModel.JointMaxVelocityRadPerSec,
                    _robotModel.JointMaxAccelerationRadPerSec2
                )
                .Subscribe(_ => Save())
                .AddTo(_disposables);
        }

        private void Save()
        {
            var dto = new RobotModelDto();
            _service.Save(dto);
        }

        public void Dispose()
        {
            _disposables.Dispose();
        }
    }
}
