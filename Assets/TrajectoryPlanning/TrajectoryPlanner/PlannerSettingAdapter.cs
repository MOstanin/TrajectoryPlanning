using TrajectoryPlanning.Services;
using TrajectoryPlanning.TrajectoryPlanner;
using Zenject;

namespace TrajectoryPlanning.Planner
{
    public sealed class PlannerSettingAdapter : IInitializable
    {
        private readonly ISettingSavingService<string, TrajectoryPlannerDto> _service;
        private readonly ITrajectoryPlanner _planner;

        public PlannerSettingAdapter(
            ISettingSavingService<string, TrajectoryPlannerDto> service,
            ITrajectoryPlanner planner
        )
        {
            _service = service;
            _planner = planner;
        }

        public void Initialize()
        {
            //Save();
        }

        private void Save()
        {
            _service.Save(new TrajectoryPlannerDto(_planner.Id, _planner.Rate));
        }
    }
}
