#ifndef __DATA_CAPTURE_PANEL_H__
#define __DATA_CAPTURE_PANEL_H__

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QtWidgets>
#endif
#include <sensor_msgs/msg/compressed_image.hpp>

namespace data_capture_rviz_plugin
{

class DataCaptureWidget : public QWidget
{
    Q_OBJECT
public:
    DataCaptureWidget(QWidget* parent=0);
};


class DataCapturePanel : public rviz_common::Panel
{
    Q_OBJECT
public:
    DataCapturePanel(QWidget* parent = nullptr);

    virtual void onInitialize();
    virtual void load(const rviz_common::Config &config);
    virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
    void tick();

protected:
    rclcpp::Node::SharedPtr nh_;
    
    DataCaptureWidget* panel_;
};

}


#endif