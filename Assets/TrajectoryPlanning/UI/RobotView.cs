using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using UnityEngine.UIElements;

namespace TrajectoryPlanning.UI
{
    public class RobotView
    {
        public Label ModelIdLabel { get; }
        public Label DofLabel { get; }
        public ScrollView JointsScroll { get; }

        private readonly List<Label> _jointValueLabels = new();

        public RobotView(UIDocument document)
        {
            var root = document.rootVisualElement;
            ModelIdLabel = root.Q<Label>("ModelIdLabel");
            DofLabel = root.Q<Label>("DofLabel");
            JointsScroll = root.Q<ScrollView>("JointsScroll");
        }

        public void SetModel(string modelId, int dof)
        {
            ModelIdLabel.text = $"Model: {modelId}";
            DofLabel.text = $"DOF: {dof}";
        }

        public void BuildJoints(string[] jointNames, int dof)
        {
            JointsScroll.Clear();
            _jointValueLabels.Clear();

            var count = Mathf.Min(jointNames.Length, dof);
            for (var i = 0; i < count; i++)
            {
                var row = new VisualElement();
                row.AddToClassList("row");

                var nameLabel = new Label(jointNames[i]);
                nameLabel.AddToClassList("jointName");

                var valueLabel = new Label("-");
                valueLabel.AddToClassList("jointValue");

                row.Add(nameLabel);
                row.Add(valueLabel);

                JointsScroll.Add(row);
                _jointValueLabels.Add(valueLabel);
            }
        }

        public void UpdateState(Vector<float> state)
        {
            var count = Mathf.Min(state.Count, _jointValueLabels.Count);
            for (var i = 0; i < count; i++)
            {
                var deg = state[i] * Mathf.Rad2Deg;
                _jointValueLabels[i].text = $"{deg:F1}°";
            }
        }
    }
}
