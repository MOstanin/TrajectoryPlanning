using System;
using UniRx;
using UnityEngine.UIElements;

namespace TrajectoryPlanning.UI
{
    public static class UIExtensions
    {
        /// <summary>
        /// Similarly <see cref="UnityEventExtensions.AsObservable"/> for ToolkitElements
        /// </summary>
        public static IObservable<Unit> AsObservable(this Clickable clickable)
        {
            return Observable.FromEvent(x => clickable.clicked += x, x => clickable.clicked -= x);
        }

        /// <summary>
        /// Observe clickable event
        /// Similarly <see cref="UnityUIComponentExtensions.OnClickAsObservable"/> for ToolkitElements
        /// </summary>
        public static IObservable<Unit> OnClickAsObservable(this Button button)
        {
            return button.clickable.AsObservable();
        }
    }
}