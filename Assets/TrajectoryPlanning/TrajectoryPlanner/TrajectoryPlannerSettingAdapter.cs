using System;
using TrajectoryPlanning.Services;
using UniRx;
using Zenject;

namespace TrajectoryPlanning.TrajectoryPlanner
{
    public sealed class TrajectoryPlannerSettingAdapter : IInitializable, IDisposable
    {
        private readonly ISettingSavingService<string, TrajectoryPlannerDto> _service;
        private readonly ITrajectoryPlanner _planner;
        private readonly CompositeDisposable _disposables = new();

        public TrajectoryPlannerSettingAdapter(
            ISettingSavingService<string, TrajectoryPlannerDto> service,
            ITrajectoryPlanner planner
        )
        {
            _service = service;
            _planner = planner;
        }

        public void Initialize()
        {
            _planner.Rate.Subscribe(_ => Save()).AddTo(_disposables);
        }

        private void Save()
        {
            _service.Save(new TrajectoryPlannerDto(_planner.Id, _planner.Rate.Value));
        }

        public void Dispose()
        {
            _disposables.Dispose();
        }
    }
}
