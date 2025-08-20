using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using UnityEngine.UIElements;

namespace TrajectoryPlanning.UI
{
    public class RobotView : IDisposable
    {
        private enum UIState
        {
            None,
            CurrentState,
            TargetState,
            Velocity,
            Acceleration
        }

        public Label ModelIdLabel { get; }
        public Label DofLabel { get; }
        public ScrollView JointsScroll { get; }
        public ScrollView TargetsScroll { get; }
        public ScrollView VelocityLimitsScroll { get; }
        public ScrollView AccelerationLimitsScroll { get; }
        public Button MoveButton { get; }
        public Button StopButton { get; }

        private readonly List<Label> _jointValueLabels = new();
        private readonly List<FloatField> _jointInputFields = new();
        private readonly List<FloatField> _velocityLimitFields = new();
        private readonly List<FloatField> _accelerationLimitFields = new();

        private readonly VisualElement _stateSection;
        private readonly VisualElement _targetsSection;
        private readonly VisualElement _velocitySection;
        private readonly VisualElement _accelerationSection;

        private readonly Toggle _stateTab;
        private readonly Toggle _targetsTab;
        private readonly Toggle _velocityTab;
        private readonly Toggle _accelerationTab;

        private readonly EventCallback<ChangeEvent<bool>> _onStateTabChanged;
        private readonly EventCallback<ChangeEvent<bool>> _onTargetsTabChanged;
        private readonly EventCallback<ChangeEvent<bool>> _onVelocityTabChanged;
        private readonly EventCallback<ChangeEvent<bool>> _onAccelerationTabChanged;

        private bool _isDisposed;

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
            StopButton = root.Q<Button>("StopButton");

            _stateSection = root.Q<VisualElement>("StateSection");
            _targetsSection = root.Q<VisualElement>("TargetsSection");
            _velocitySection = root.Q<VisualElement>("VelocitySection");
            _accelerationSection = root.Q<VisualElement>("AccelerationSection");

            _stateTab = root.Q<Toggle>("StateTab");
            _targetsTab = root.Q<Toggle>("TargetsTab");
            _velocityTab = root.Q<Toggle>("VelocityTab");
            _accelerationTab = root.Q<Toggle>("AccelerationTab");

            _onStateTabChanged = evt =>
            {
                if (evt.newValue)
                    ShowSection(UIState.CurrentState);
            };
            _onTargetsTabChanged = evt =>
            {
                if (evt.newValue)
                    ShowSection(UIState.TargetState);
            };
            _onVelocityTabChanged = evt =>
            {
                if (evt.newValue)
                    ShowSection(UIState.Velocity);
            };
            _onAccelerationTabChanged = evt =>
            {
                if (evt.newValue)
                    ShowSection(UIState.Acceleration);
            };

            _stateTab.RegisterValueChangedCallback(_onStateTabChanged);
            _targetsTab.RegisterValueChangedCallback(_onTargetsTabChanged);
            _velocityTab.RegisterValueChangedCallback(_onVelocityTabChanged);
            _accelerationTab.RegisterValueChangedCallback(_onAccelerationTabChanged);

            ShowSection(UIState.CurrentState);
        }

        private void ShowSection(UIState which)
        {
            _stateSection.style.display =
                which == UIState.CurrentState ? DisplayStyle.Flex : DisplayStyle.None;
            _targetsSection.style.display =
                which == UIState.TargetState ? DisplayStyle.Flex : DisplayStyle.None;
            _velocitySection.style.display =
                which == UIState.Velocity ? DisplayStyle.Flex : DisplayStyle.None;
            _accelerationSection.style.display =
                which == UIState.Acceleration ? DisplayStyle.Flex : DisplayStyle.None;

            _stateTab.SetValueWithoutNotify(which == UIState.CurrentState);
            _targetsTab.SetValueWithoutNotify(which == UIState.TargetState);
            _velocityTab.SetValueWithoutNotify(which == UIState.Velocity);
            _accelerationTab.SetValueWithoutNotify(which == UIState.Acceleration);
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

        public void Dispose()
        {
            _stateTab.UnregisterValueChangedCallback(_onStateTabChanged);
            _targetsTab.UnregisterValueChangedCallback(_onTargetsTabChanged);
            _velocityTab.UnregisterValueChangedCallback(_onVelocityTabChanged);
            _accelerationTab.UnregisterValueChangedCallback(_onAccelerationTabChanged);
        }
    }
}
