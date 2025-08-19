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

        public override void InstallBindings()
        {
            if (robotUiDocument == null)
                throw new Exception("Failed to load robot UI document");

            Container.BindInstance(new RobotView(robotUiDocument)).AsSingle();
            Container.BindInterfacesAndSelfTo<RobotUIPresenter>().AsSingle();
        }
    }
}
