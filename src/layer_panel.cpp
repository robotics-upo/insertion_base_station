#include "insertion_base_station/layer_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display.hpp>

namespace insertion_base_station {

LayerPanel::LayerPanel(QWidget* parent) : rviz_common::Panel(parent) {
    QVBoxLayout* layout = new QVBoxLayout;
    
    QLabel* title = new QLabel("<b>Visibility Layers</b>");
    layout->addWidget(title);

    // Initialize checkboxes (Defaulted to checked/visible)
    cb_radar1_ = new QCheckBox("Drone 1: Radar");
    cb_radar1_->setChecked(true);
    layout->addWidget(cb_radar1_);

    cb_radar2_ = new QCheckBox("Drone 2: Radar");
    cb_radar2_->setChecked(true);
    layout->addWidget(cb_radar2_);

    cb_paths_ = new QCheckBox("Flight Paths");
    cb_paths_->setChecked(true);
    layout->addWidget(cb_paths_);

    cb_labels_ = new QCheckBox("Mission Tags");
    cb_labels_->setChecked(true);
    layout->addWidget(cb_labels_);

    cb_tfs_ = new QCheckBox("TF Frames");
    cb_tfs_->setChecked(true);
    layout->addWidget(cb_tfs_);

    cb_interactive_ = new QCheckBox("Edit Waypoints (Unlock)");
    cb_interactive_->setChecked(false); 
    layout->addWidget(cb_interactive_);
    
    layout->addStretch(); // Pushes UI elements to the top
    setLayout(layout);

    // Modern Qt connect syntax for slots
    connect(cb_radar1_,  &QCheckBox::toggled, this, &LayerPanel::onRadar1Toggled);
    connect(cb_radar2_,  &QCheckBox::toggled, this, &LayerPanel::onRadar2Toggled);
    connect(cb_paths_,   &QCheckBox::toggled, this, &LayerPanel::onPathsToggled);
    connect(cb_labels_,  &QCheckBox::toggled, this, &LayerPanel::onLabelsToggled);
    connect(cb_tfs_,     &QCheckBox::toggled, this, &LayerPanel::onTFsToggled);
    connect(cb_interactive_, &QCheckBox::toggled, this, &LayerPanel::onInteractiveToggled);
}

void LayerPanel::onInitialize() {
    // RViz initialization is complete. 
    // We force the UI state to match the RViz display state at startup.
    onRadar1Toggled(cb_radar1_->isChecked());
    onRadar2Toggled(cb_radar2_->isChecked());
    onPathsToggled(cb_paths_->isChecked());
    onTFsToggled(cb_tfs_->isChecked());
    onLabelsToggled(cb_labels_->isChecked());
    onInteractiveToggled(cb_interactive_->isChecked());
}

// ==========================================
// TOGGLE SLOTS
// ==========================================
// WARNING: The string names MUST MATCH EXACTLY the display names in the RViz tree.
void LayerPanel::onRadar1Toggled(bool checked) { setDisplayEnabled("Drone1_radar", checked); }
void LayerPanel::onRadar2Toggled(bool checked) { setDisplayEnabled("Drone2_radar", checked); }

void LayerPanel::onPathsToggled(bool checked) { 
    setDisplayEnabled("Drone1_path", checked); 
    setDisplayEnabled("Drone2_path", checked); 
    setDisplayEnabled("Boat_path", checked);
}

void LayerPanel::onLabelsToggled(bool checked) { 
    setDisplayEnabled("Tags", checked); 
}

void LayerPanel::onTFsToggled(bool checked) { 
    setDisplayEnabled("TF", checked); 
}

void LayerPanel::onInteractiveToggled(bool checked) {
    setDisplayEnabled("WP_interactive", checked);
}

// ==========================================
// RVIZ DISPLAY TREE SEARCH LOGIC
// ==========================================
void LayerPanel::setDisplayEnabled(const QString& name, bool enabled) {
    auto context = getDisplayContext();
    if (!context) return;
    
    // Get the root group ("Displays" panel in RViz)
    rviz_common::DisplayGroup* root = context->getRootDisplayGroup();
    if (!root) return;

    // Start recursive search
    searchAndToggleDisplay(root, name, enabled);
}

void LayerPanel::searchAndToggleDisplay(rviz_common::DisplayGroup* group, const QString& name, bool enabled) {
    if (!group) return;

    // Iterate through all items in the current group/folder
    for (int i = 0; i < group->numDisplays(); ++i) {
        rviz_common::Display* display = group->getDisplayAt(i);
        if (!display) continue;

        // If the name matches, toggle visibility
        if (display->getName() == name) {
            display->setEnabled(enabled);
        }

        // Magic trick: Check if this display is actually a Group/Folder
        // If it is, recursively search inside it!
        rviz_common::DisplayGroup* sub_group = dynamic_cast<rviz_common::DisplayGroup*>(display);
        if (sub_group) {
            searchAndToggleDisplay(sub_group, name, enabled);
        }
    }
}

} // namespace insertion_base_station

PLUGINLIB_EXPORT_CLASS(insertion_base_station::LayerPanel, rviz_common::Panel)