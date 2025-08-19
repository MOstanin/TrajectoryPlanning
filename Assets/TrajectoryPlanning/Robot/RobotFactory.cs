using System;
using TrajectoryPlanning.Services;
using UnityEngine;
using Zenject;

namespace TrajectoryPlanning.Robot
{
    public class RobotFactory : IFactory<string, RobotModel>
    {
        private readonly ISettingSavingService<string, RobotModelDto> _settings;

        public RobotFactory(ISettingSavingService<string, RobotModelDto> settings)
        {
            _settings = settings;
        }

        public RobotModel Create(string id)
        {
            var loaded = _settings.Load(id);
            var config = loaded.Match(
                some => some,
                none => throw new Exception($"Failed to load robot model: {id}")
            );

            var robotModel = new RobotModel(config);

            Debug.Log($"Created robot model {robotModel.Id}");
            return robotModel;
        }
    }
}
