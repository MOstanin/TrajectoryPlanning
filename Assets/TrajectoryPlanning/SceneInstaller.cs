using Newtonsoft.Json;
using TrajectoryPlanning.Robot;
using TrajectoryPlanning.Services;
using Zenject;

public class SceneInstaller : MonoInstaller
{
    public override void InstallBindings()
    {
        Container.BindInstance(new JsonSerializer()).IfNotBound();
        Container
            .Bind<ISettingSavingService<string, RobotModelDto>>()
            .To<SettingSavingService<string, RobotModelDto>>()
            .AsSingle()
            .WithArguments("Robot/");
        Container.Bind<RobotSettingAdapter>().AsSingle();
        Container.BindIFactory<string, RobotModel>().FromFactory<RobotFactory>();
        Container
            .BindInterfacesTo<RobotModel>()
            .FromResolveGetter<IFactory<string, RobotModel>, RobotModel>(
                factory => factory.Create("UR10e")
            )
            .AsSingle();
    }
}
