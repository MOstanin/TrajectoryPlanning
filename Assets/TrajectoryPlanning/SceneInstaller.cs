using Newtonsoft.Json;
using TrajectoryPlanning.Robot;
using TrajectoryPlanning.Services;
using TrajectoryPlanning.TrajectoryPlanner;
using Zenject;

namespace TrajectoryPlanning
{
    public class SceneInstaller : MonoInstaller
    {
        public override void InstallBindings()
        {
            Container.BindInstance(new JsonSerializer());
            Container
                .Bind<ISettingSavingService<string, RobotModelDto>>()
                .To<SettingSavingService<string, RobotModelDto>>()
                .AsSingle()
                .WithArguments("Robot");
            Container.BindIFactory<string, RobotModel>().FromFactory<RobotFactory>();
            Container
                .BindInterfacesTo<RobotModel>()
                .FromResolveGetter<IFactory<string, RobotModel>, RobotModel>(
                    factory => factory.Create("UR10e")
                )
                .AsSingle();
            Container.BindInterfacesTo<RobotSettingAdapter>().AsSingle().NonLazy();

            Container
                .Bind<ISettingSavingService<string, TrajectoryPlannerDto>>()
                .To<SettingSavingService<string, TrajectoryPlannerDto>>()
                .AsSingle()
                .WithArguments("Planner");
            Container
                .BindIFactory<string, TrapezoidalTrajectoryPlanner>()
                .FromFactory<TrajectoryPlannerFactory>();
            Container
                .BindInterfacesTo<TrapezoidalTrajectoryPlanner>()
                .FromResolveGetter<
                    IFactory<string, TrapezoidalTrajectoryPlanner>,
                    TrapezoidalTrajectoryPlanner
                >(factory => factory.Create("TrapezoidalPlanner"))
                .AsSingle();
            Container.BindInterfacesTo<TrajectoryPlannerSettingAdapter>().AsSingle().NonLazy();

            Container.BindInterfacesTo<RobotController>().AsSingle();
        }
    }
}
