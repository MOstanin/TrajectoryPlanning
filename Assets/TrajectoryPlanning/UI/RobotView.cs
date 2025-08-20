using System;
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
        public ScrollView VelocityLimitsScroll { get; }
        public ScrollView AccelerationLimitsScroll { get; }
        public Button MoveButton { get; }

        private readonly List<Label> _jointValueLabels = new();
        private readonly List<FloatField> _jointInputFields = new();
        private readonly List<FloatField> _velocityLimitFields = new();
        private readonly List<FloatField> _accelerationLimitFields = new();

        public RobotView(UIDocument document)
        {
            var root = document.rootVisualElement;
            ModelIdLabel = root.Q<Label>("ModelIdLabel");
            DofLabel = root.Q<Label>("DofLabel");
            JointsScroll = root.Q<ScrollView>("JointsScroll");
            TargetsScroll = root.Q<ScrollView>("TargetsScroll");
            VelocityLimitsScroll = root.Q<ScrollView>("VelocityLimitsScroll");
            AccelerationLimitsScroll = root.Q<ScrollView>("AccelerationLimitsScroll");
            MoveButton = root.Q<Button>("MoveButton");

            var stateSection = root.Q<VisualElement>("StateSection");
            var targetsSection = root.Q<VisualElement>("TargetsSection");
            var velocitySection = root.Q<VisualElement>("VelocitySection");
            var accelerationSection = root.Q<VisualElement>("AccelerationSection");

            var stateTab = root.Q<Toggle>("StateTab");
            var targetsTab = root.Q<Toggle>("TargetsTab");
            var velocityTab = root.Q<Toggle>("VelocityTab");
            var accelerationTab = root.Q<Toggle>("AccelerationTab");

            stateTab.RegisterValueChangedCallback(evt =>
            {
                if (evt.newValue)
                    ShowSection("State");
            });
            targetsTab.RegisterValueChangedCallback(evt =>
            {
                if (evt.newValue)
                    ShowSection("Targets");
            });

            velocityTab.RegisterValueChangedCallback(evt =>
            {
                if (evt.newValue)
                    ShowSection("Velocity");
            });
            accelerationTab.RegisterValueChangedCallback(evt =>
            {
                if (evt.newValue)
                    ShowSection("Acceleration");
            });

            ShowSection("State");
            return;

            void ShowSection(string which)
            {
                stateSection.style.display =
                    which == "State" ? DisplayStyle.Flex : DisplayStyle.None;
                targetsSection.style.display =
                    which == "Targets" ? DisplayStyle.Flex : DisplayStyle.None;
                velocitySection.style.display =
                    which == "Velocity" ? DisplayStyle.Flex : DisplayStyle.None;
                accelerationSection.style.display =
                    which == "Acceleration" ? DisplayStyle.Flex : DisplayStyle.None;

                stateTab.SetValueWithoutNotify(which == "State");
                targetsTab.SetValueWithoutNotify(which == "Targets");
                velocityTab.SetValueWithoutNotify(which == "Velocity");
                accelerationTab.SetValueWithoutNotify(which == "Acceleration");
            }
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

        public void BuildLimitEditors(string[] jointNames, int dof)
        {
            VelocityLimitsScroll.Clear();
            AccelerationLimitsScroll.Clear();
            _velocityLimitFields.Clear();
            _accelerationLimitFields.Clear();

            var count = Mathf.Min(jointNames.Length, dof);
            for (var i = 0; i < count; i++)
            {
                var velRow = new VisualElement();
                velRow.AddToClassList("row");
                var velName = new Label(jointNames[i]);
                velName.AddToClassList("jointName");
                var velField = new FloatField();
                velField.AddToClassList("jointInput");
                velRow.Add(velName);
                velRow.Add(velField);
                VelocityLimitsScroll.Add(velRow);
                _velocityLimitFields.Add(velField);

                var accRow = new VisualElement();
                accRow.AddToClassList("row");
                var accName = new Label(jointNames[i]);
                accName.AddToClassList("jointName");
                var accField = new FloatField();
                accField.AddToClassList("jointInput");
                accRow.Add(accName);
                accRow.Add(accField);
                AccelerationLimitsScroll.Add(accRow);
                _accelerationLimitFields.Add(accField);
            }
        }

        public void SetVelocityLimitsDegPerSec(float[] valuesRadPerSec)
        {
            var count = Mathf.Min(valuesRadPerSec.Length, _velocityLimitFields.Count);
            for (var i = 0; i < count; i++)
            {
                var deg = valuesRadPerSec[i] * Mathf.Rad2Deg;
                _velocityLimitFields[i].SetValueWithoutNotify(deg);
            }
        }

        public void SetAccelerationLimitsDegPerSec2(float[] valuesRadPerSec2)
        {
            var count = Mathf.Min(valuesRadPerSec2.Length, _accelerationLimitFields.Count);
            for (var i = 0; i < count; i++)
            {
                var deg = valuesRadPerSec2[i] * Mathf.Rad2Deg;
                _accelerationLimitFields[i].SetValueWithoutNotify(deg);
            }
        }

        public void ForEachVelocityLimitField(Action<int, FloatField> action)
        {
            for (var i = 0; i < _velocityLimitFields.Count; i++)
                action(i, _velocityLimitFields[i]);
        }

        public void ForEachAccelerationLimitField(Action<int, FloatField> action)
        {
            for (var i = 0; i < _accelerationLimitFields.Count; i++)
                action(i, _accelerationLimitFields[i]);
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
