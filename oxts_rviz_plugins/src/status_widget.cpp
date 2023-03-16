#include <oxts_rviz_plugins/status_widget.hpp>

namespace oxts_rviz_plugins {

StatusWidget::StatusWidget(QWidget* parent) : QWidget(parent) {
	auto options = rclcpp::NodeOptions();
	_node = std::make_shared<rclcpp::Node>("oxts_plugin", options);
	subNcom_ = _node->create_subscription<oxts_msgs::msg::Ncom>(
		subTopic, 1, std::bind(&StatusWidget::ncomCallback, this, std::placeholders::_1));

	RCLCPP_INFO(_node->get_logger(), "Subscribing to %s", subTopic.c_str());

	initLayout();
    connect(TopicEditor, SIGNAL(editingFinished()), this, SLOT(updateTopic()));

	nrx = NComCreateNComRxC();
}

StatusWidget::~StatusWidget() {
    NComDestroyNComRxC(nrx);
    nrx = NULL;
}

// Updates subscription topic from user input
void StatusWidget::updateTopic() {
    auto newTopic = TopicEditor->text().toStdString();
    if(newTopic == subTopic) 
        return;
    if(newTopic == "") {
        RCLCPP_WARN(_node->get_logger(), "Can't set subscription to empty topic!");    
        return;
    }

    subTopic = newTopic;
    subNcom_ = _node->create_subscription<oxts_msgs::msg::Ncom>(
        subTopic, 1, std::bind(&StatusWidget::ncomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(_node->get_logger(), "Subscribing to %s", subTopic.c_str());
}

void StatusWidget::initLayout() {
    TopicEditor = new QLineEdit(this);
    TopicEditor->setPlaceholderText("/ins/ncom");

    TopicLabel = new QTextLabel(this, "NCOM Topic:");
	NavLabel = new QTextLabel(this, "Navigation system mode:");
	EastAccLabel = new QTextLabel(this, "East accuracy:");
	NorthAccLabel = new QTextLabel(this, "North accuracy:");
	UpAccLabel = new QTextLabel(this, "Up accuracy:");
	RollAccLabel = new QTextLabel(this, "Roll accuracy:");
	PitchAccLabel = new QTextLabel(this, "Pitch accuracy:");
	YawAccLabel = new QTextLabel(this, "Yaw accuracy:");
	SatLabelPrimary = new QTextLabel(this, "Primary antenna #satellites tracked:");
	SatLabelSecondary = new QTextLabel(this, "Secondary antenna #satellites tracked:");

	NorthAccStatus = new QDoubleLabel(this);
	EastAccStatus = new QDoubleLabel(this);
	UpAccStatus = new QDoubleLabel(this);
	RollAccStatus = new QDoubleLabel(this);
	PitchAccStatus = new QDoubleLabel(this);
	YawAccStatus = new QDoubleLabel(this);

	SatStatusPrimary = new QSatStatusLabel(this);
	SatStatusSecondary = new QSatStatusLabel(this);

	NavStatus = new QNavStatusLabel(this);

	RollAccStatus->setLimits(0.2, 0.5);
	PitchAccStatus->setLimits(0.2, 0.5);
	YawAccStatus->setLimits(0.2, 0.5);

	QGridLayout* grid = new QGridLayout(this);

    grid->addWidget(TopicLabel, 0, 0);
    grid->addWidget(TopicEditor, 0, 1);
	grid->addWidget(NavLabel, 1, 0);
	grid->addWidget(NavStatus, 1, 1);
	grid->addWidget(EastAccLabel, 2, 0);
	grid->addWidget(EastAccStatus, 2, 1);
	grid->addWidget(NorthAccLabel, 3, 0);
	grid->addWidget(NorthAccStatus, 3, 1);
	grid->addWidget(UpAccLabel, 4, 0);
	grid->addWidget(UpAccStatus, 4, 1);
	grid->addWidget(RollAccLabel, 5, 0);
	grid->addWidget(RollAccStatus, 5, 1);
	grid->addWidget(PitchAccLabel, 6, 0);
	grid->addWidget(PitchAccStatus, 6, 1);
	grid->addWidget(YawAccLabel, 7, 0);
	grid->addWidget(YawAccStatus, 7, 1);
	grid->addWidget(SatLabelPrimary, 8, 0);
	grid->addWidget(SatStatusPrimary, 8, 1);
	grid->addWidget(SatLabelSecondary, 9, 0);
	grid->addWidget(SatStatusSecondary, 9, 1);

	setLayout(grid);
}

void StatusWidget::ncomCallback(const oxts_msgs::msg::Ncom::SharedPtr msg) {
	if (NComNewChars(this->nrx, msg->raw_packet.data(), NCOM_PACKET_LENGTH) == COM_NEW_UPDATE) {
		if (this->nrx->mIsEastAccValid)
			EastAccStatus->setValue(this->nrx->mEastAcc);
		if (this->nrx->mIsNorthAccValid)
			NorthAccStatus->setValue(this->nrx->mNorthAcc);
		if (this->nrx->mIsAltAccValid)
			UpAccStatus->setValue(this->nrx->mAltAcc);
		if (this->nrx->mIsRollAccValid)
			RollAccStatus->setValue(this->nrx->mRollAcc);
		if (this->nrx->mIsPitchAccValid)
			PitchAccStatus->setValue(this->nrx->mPitchAcc);
		if (this->nrx->mIsHeadingAccValid)
			YawAccStatus->setValue(this->nrx->mHeadingAcc);
		if (this->nrx->mIsInsNavModeValid)
			NavStatus->setValue(this->nrx->mInsNavMode);
		if (this->nrx->mGpsPrimary->mIsNumSatsValid)
			SatStatusPrimary->setValue(this->nrx->mGpsPrimary->mNumSats);
		if (this->nrx->mGpsSecondary->mIsNumSatsValid)
			SatStatusSecondary->setValue(this->nrx->mGpsSecondary->mNumSats);
	}
}
}  // namespace oxts_rviz_plugins
