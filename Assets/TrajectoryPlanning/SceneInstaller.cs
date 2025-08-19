using TrajectoryPlanning.Robot;
using UnityEngine;
using Zenject;

public class SceneInstaller : MonoInstaller
{
    public override void InstallBindings()
    {
        Container
            .BindInterfacesTo<RobotModel>()
            .AsSingle()
            .WithArguments(new[] { 0, 0, -Mathf.PI / 2, 0, 0, 0 });
    }
}
