using System;
using UnityEngine;
using UnityEngine.UIElements;
using Zenject;

namespace TrajectoryPlanning.UI
{
    public class UIInstaller : MonoInstaller
    {
        [SerializeField]
        private UIDocument? robotUiDocument;

        [SerializeField]
        private UIDocument? plannerUiDocument;

        public override void InstallBindings()
        {
            if (robotUiDocument == null)
                throw new Exception("Failed to load robot UI document");
            if (plannerUiDocument == null)
                throw new Exception("Failed to load planner UI document");

            Container
                .BindInterfacesAndSelfTo<RobotView>()
                .AsSingle()
                .WithArguments(robotUiDocument);
            Container.BindInterfacesAndSelfTo<RobotUIPresenter>().AsSingle();

            Container
                .BindInterfacesAndSelfTo<TrajectoryPlannerView>()
                .AsSingle()
                .WithArguments(plannerUiDocument);
            Container.BindInterfacesAndSelfTo<TrajectoryPlannerPresenter>().AsSingle();
        }
    }
}
