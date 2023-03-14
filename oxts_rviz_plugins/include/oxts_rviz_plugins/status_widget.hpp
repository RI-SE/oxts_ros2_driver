#pragma once

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

#include <memory>
#include <vector>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include "oxts_msgs/msg/ncom.hpp"
#include "oxts_rviz_plugins/NComRxC.h"
#include "oxts_rviz_plugins/QNavStatusLabels.hpp"
#endif

namespace oxts_rviz_plugins {
class StatusWidget : public QWidget {
	Q_OBJECT
   public:
	StatusWidget(QWidget* parent = 0);
	~StatusWidget() override;

	rclcpp::Node::SharedPtr _node;

   public Q_SLOTS:
   private Q_SLOTS:

   protected:
	void ncomCallback(const oxts_msgs::msg::Ncom::SharedPtr msg);

   private:
	rclcpp::Subscription<oxts_msgs::msg::Ncom>::SharedPtr subNcom_;

	QLabel* NavLabel;
	QNavStatusLabel* NavStatus;
	QLabel* EastAccLabel;
	QDoubleLabel* EastAccStatus;
	QLabel* NorthAccLabel;
	QDoubleLabel* NorthAccStatus;
	QLabel* UpAccLabel;
	QDoubleLabel* UpAccStatus;

	NComRxC* nrx;
};
}  // namespace oxts_rviz_plugins