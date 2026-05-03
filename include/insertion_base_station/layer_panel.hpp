#ifndef LAYER_PANEL_HPP
#define LAYER_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <QtWidgets>

// Forward declaration to avoid including full headers in .hpp
namespace rviz_common {
    class DisplayGroup;
}

namespace insertion_base_station {

class LayerPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit LayerPanel(QWidget * parent = nullptr);
  virtual void onInitialize() override;

protected Q_SLOTS:
  // Slots triggered when checkboxes are toggled
  void onRadar1Toggled(bool checked);
  void onRadar2Toggled(bool checked);
  void onPathsToggled(bool checked);
  void onLabelsToggled(bool checked);
  void onTFsToggled(bool checked);
  void onInteractiveToggled(bool checked);

private:
  // UI Checkboxes
  QCheckBox* cb_radar1_;
  QCheckBox* cb_radar2_;
  QCheckBox* cb_paths_;
  QCheckBox* cb_labels_;
  QCheckBox* cb_tfs_;
  QCheckBox* cb_interactive_;

  // Master function to find a display in RViz and toggle its visibility
  void setDisplayEnabled(const QString& display_name, bool enabled);
  
  // Recursive helper to search inside grouped folders in RViz
  void searchAndToggleDisplay(rviz_common::DisplayGroup* group, const QString& name, bool enabled);
};

} // namespace insertion_base_station

#endif // LAYER_PANEL_HPP