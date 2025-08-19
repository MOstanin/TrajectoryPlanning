using Newtonsoft.Json;
using TrajectoryPlanning.Robot;
using TrajectoryPlanning.Services;
using UnityEngine;
using Zenject;

public class SceneInstaller : MonoInstaller
{
    public override void InstallBindings()
    {
        Container.BindInstance(new JsonSerializer()).IfNotBound();
        Container
            .Bind<ISettingSavingService<RobotModelDto>>()
            .To<SettingSavingService<RobotModelDto>>()
            .AsSingle()
            .WithArguments("Robot/", "Robot/");
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
