using UnityEngine.UIElements;

namespace TrajectoryPlanning.UI
{
    public class TrajectoryPlannerView
    {
        public Label PlannerIdLabel { get; }
        public FloatField RateField { get; }

        public TrajectoryPlannerView(UIDocument document)
        {
            var root = document.rootVisualElement;
            PlannerIdLabel = root.Q<Label>("PlannerIdLabel");
            RateField = root.Q<FloatField>("RateField");
        }

        public void SetPlanner(string plannerId, float rate)
        {
            PlannerIdLabel.text = $"Planner: {plannerId}";
            RateField.value = rate;
        }
    }
}


