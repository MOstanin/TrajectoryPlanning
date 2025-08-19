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
        public ScrollView TargetsScroll { get; }
        public Button MoveButton { get; }

        private readonly List<Label> _jointValueLabels = new();
        private readonly List<FloatField> _jointInputFields = new();

        public RobotView(UIDocument document)
        {
            var root = document.rootVisualElement;
            ModelIdLabel = root.Q<Label>("ModelIdLabel");
            DofLabel = root.Q<Label>("DofLabel");
            JointsScroll = root.Q<ScrollView>("JointsScroll");
            TargetsScroll = root.Q<ScrollView>("TargetsScroll");
            MoveButton = root.Q<Button>("MoveButton");
        }

        public void SetModel(string modelId, int dof)
        {
            ModelIdLabel.text = $"Model: {modelId}";
            DofLabel.text = $"DOF: {dof}";
        }

        public void BuildJoints(string[] jointNames, int dof)
        {
            JointsScroll.Clear();
            TargetsScroll.Clear();
            _jointValueLabels.Clear();
            _jointInputFields.Clear();

            var count = Mathf.Min(jointNames.Length, dof);
            for (var i = 0; i < count; i++)
            {
                var valueRow = new VisualElement();
                valueRow.AddToClassList("row");

                var nameLabel = new Label(jointNames[i]);
                nameLabel.AddToClassList("jointName");

                var valueLabel = new Label("-");
                valueLabel.AddToClassList("jointValue");

                valueRow.Add(nameLabel);
                valueRow.Add(valueLabel);

                JointsScroll.Add(valueRow);
                _jointValueLabels.Add(valueLabel);

                var targetRow = new VisualElement();
                targetRow.AddToClassList("row");

                var targetName = new Label(jointNames[i]);
                targetName.AddToClassList("jointName");

                var input = new FloatField();
                input.AddToClassList("jointInput");

                targetRow.Add(targetName);
                targetRow.Add(input);
                TargetsScroll.Add(targetRow);
                _jointInputFields.Add(input);
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

        public Vector<float> GetTargetStateRadians(int dof)
        {
            var count = Mathf.Min(dof, _jointInputFields.Count);
            var vec = Vector<float>.Build.Dense(dof);
            for (var i = 0; i < count; i++)
            {
                var deg = _jointInputFields[i].value;
                vec[i] = deg * Mathf.Deg2Rad;
            }
            return vec;
        }
    }
}
