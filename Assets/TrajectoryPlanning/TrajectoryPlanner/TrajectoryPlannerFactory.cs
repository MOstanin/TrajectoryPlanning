using System;
using TrajectoryPlanning.Services;
using UnityEngine;
using Zenject;

namespace TrajectoryPlanning.TrajectoryPlanner
{
    public class TrajectoryPlannerFactory : IFactory<string, TrapezoidalTrajectoryPlanner>
    {
        private readonly ISettingSavingService<string, TrajectoryPlannerDto> _settings;

        public TrajectoryPlannerFactory(
            ISettingSavingService<string, TrajectoryPlannerDto> settings
        )
        {
            _settings = settings;
        }

        public TrapezoidalTrajectoryPlanner Create(string id)
        {
            var loaded = _settings.Load(id);
            var config = loaded.Match(
                some => some,
                none => throw new Exception($"Failed to load planner model: {id}")
            );

            var planner = new TrapezoidalTrajectoryPlanner(config.Id, config.rate);
            Debug.Log($"Created trajectory planner {config.Id}");
            return planner;
        }
    }
}
