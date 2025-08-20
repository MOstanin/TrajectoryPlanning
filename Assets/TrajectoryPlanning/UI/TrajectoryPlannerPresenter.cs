using System;
using TrajectoryPlanning.TrajectoryPlanner;
using UniRx;
using UnityEngine.UIElements;
using Zenject;

namespace TrajectoryPlanning.UI
{
    public class TrajectoryPlannerPresenter : IInitializable, IDisposable
    {
        private readonly ITrajectoryPlanner _planner;
        private readonly TrajectoryPlannerView _view;
        private readonly CompositeDisposable _disposables = new();

        public TrajectoryPlannerPresenter(ITrajectoryPlanner planner, TrajectoryPlannerView view)
        {
            _planner = planner;
            _view = view;
        }

        public void Initialize()
        {
            _view.SetPlanner(_planner.Id, _planner.Rate.Value);

            _planner
                .Rate.Subscribe(v => _view.RateField.SetValueWithoutNotify(v))
                .AddTo(_disposables);

            EventCallback<ChangeEvent<float>> eventCallback = evt =>
            {
                if (evt.newValue is var newRate)
                    _planner.Rate.Value = newRate;
            };
            _view.RateField.RegisterValueChangedCallback(eventCallback);
            Disposable
                .Create(() => _view.RateField.UnregisterValueChangedCallback(eventCallback))
                .AddTo(_disposables);
        }

        public void Dispose()
        {
            _disposables.Dispose();
        }
    }
}
