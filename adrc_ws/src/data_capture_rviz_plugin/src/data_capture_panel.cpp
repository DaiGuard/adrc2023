#include "data_capture_panel.h"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

namespace data_capture_rviz_plugin
{

DataCaptureWidget::DataCaptureWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(100,100);
    setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
}

DataCapturePanel::DataCapturePanel(QWidget* parent)
    : rviz_common::Panel(parent)
{
    QVBoxLayout* layout = new QVBoxLayout;

    panel_ = new DataCaptureWidget();
    layout->addWidget(panel_);
    setLayout(layout);
    panel_->update();
}

void DataCapturePanel::onInitialize()
{
    nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void DataCapturePanel::tick()
{

}

void DataCapturePanel::load(const rviz_common::Config &config)
{
    rviz_common::Panel::load(config);
}

void DataCapturePanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(data_capture_rviz_plugin::DataCapturePanel, rviz_common::Panel)