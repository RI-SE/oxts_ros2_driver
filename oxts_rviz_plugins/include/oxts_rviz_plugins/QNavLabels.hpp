#pragma once

#include <QLabel>
#include <QWidget>

#include "oxts_rviz_plugins/nav_const.hpp"

enum class Status { UNKNOWN, OK, WARNING, ERROR };

struct GpsLimits {
	int green = 9;
	int yellow = 5;
	int red = 0;
};

struct DoubleLimit {
	double green = 0.05;
	double yellow = 0.1;
	double red = __DBL_MAX__;
};

/**
 * @brief QLabel with a default style
 */
class QInitializedLabel : public QLabel {
	Q_OBJECT
   public:
	explicit QInitializedLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QLabel(parent, f);
		initStyle();
	}

	virtual void initStyle() {
		this->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
		this->setWordWrap(true);
	}

	virtual ~QInitializedLabel(){};
};

/**
 * @brief QLabel with a default style and a status value
 */
class QStatusLabel : public QInitializedLabel {
	Q_OBJECT
   public:
	explicit QStatusLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QInitializedLabel(parent, f);
		initStyle();
	}

	virtual void initStyle() override {
		QInitializedLabel::initStyle();
		this->setText("Awaiting data");
		this->setStyleSheet("QLabel { background-color : red; color : black; }");
		status = Status::UNKNOWN;
	}

	virtual ~QStatusLabel(){};

	Status status = Status::UNKNOWN;
};

/**
 * @brief QLabel with a default style and a possibility to construct it with string input
 */
class QTextLabel : public QInitializedLabel {
	Q_OBJECT
   public:
	explicit QTextLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QInitializedLabel(parent, f);
	}

	explicit QTextLabel(QWidget* parent, QString text, Qt::WindowFlags f = Qt::WindowFlags()) {
		QInitializedLabel(parent, f);
		this->setText(text);
	}

	virtual ~QTextLabel(){};
};

/**
 * @brief QLabel to hold information about the number of satellites
 */
class QSatStatusLabel : public QStatusLabel {
	Q_OBJECT

   public:
	explicit QSatStatusLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QInitializedLabel(parent, f);
	}
	virtual ~QSatStatusLabel(){};

	/**
	 * @brief Set the value of the label and updates the style based on value
	 * @param nSats Number of satellites
	 */
	void setValue(const int nSats) {
		this->setText(QString::number(nSats));
		this->updateStyle(nSats);
	}

   private:
	/**
	 * @brief Update the style of the label based on the value
	 * @param d Number of satellites
	 */
	void updateStyle(const int d) {
		auto newStatus = evaluateStatus(d);
		if (status == newStatus) {
			return;
		}

		status = newStatus;
		if (status == Status::OK) {
			this->setStyleSheet("background-color : green;");
		} else if (status == Status::WARNING) {
			this->setStyleSheet("background-color : yellow;");
		} else if (status == Status::ERROR) {
			this->setStyleSheet("background-color : red;");
		}
	}

	Status evaluateStatus(const int d) {
		if (d > limit.green) {
			return Status::OK;
		} else if (d >= limit.yellow && d < limit.green) {
			return Status::WARNING;
		} else {
			return Status::ERROR;
		}
	}

	GpsLimits limit;
};

/**
 * @brief QLabel to hold information about the navigation mode
 */
class QNavStatusLabel : public QStatusLabel {
	Q_OBJECT
   public:
	explicit QNavStatusLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QInitializedLabel(parent, f);
	}
	virtual ~QNavStatusLabel(){};

	/**
	 * @brief Set the value of the label and updates the style based on value
	 * @param nav Navigation status
	 */
	void setValue(const uint8_t nav) {
		if (status == getNavMode(nav)) {
			return;
		}

		status = getNavMode(nav);
		switch (status) {
		case NAV_CONST::NAV_MODE::REAL_TIME:
			this->setStyleSheet("background-color : green;");
			this->setText("REAL_TIME");
			break;
		case NAV_CONST::NAV_MODE::READY_TO_INITIALISE:
			this->setStyleSheet("background-color : blue;");
			this->setText("READY_TO_INITIALISE");
			break;
		case NAV_CONST::NAV_MODE::LOCKING_ON:
			this->setStyleSheet("background-color : yellow;");
			this->setText("LOCKING_ON");
			break;
		case NAV_CONST::NAV_MODE::RAW_INERTIAL_DATA:
			this->setStyleSheet("background-color : red;");
			this->setText("RAW_INERTIAL_DATA");
			break;
		default:
			break;
		}
	}

	virtual void initStyle() override {
		QStatusLabel::initStyle();
		status = NAV_CONST::NAV_MODE::RAW_INERTIAL_DATA;
	}

   private:
	NAV_CONST::NAV_MODE getNavMode(uint8_t mode) {
		switch (mode) {
		case 1:
			return NAV_CONST::NAV_MODE::RAW_INERTIAL_DATA;
		case 2:
			return NAV_CONST::NAV_MODE::READY_TO_INITIALISE;
		case 3:
			return NAV_CONST::NAV_MODE::LOCKING_ON;
		case 4:
			return NAV_CONST::NAV_MODE::REAL_TIME;
		default:
			return NAV_CONST::NAV_MODE::RAW_INERTIAL_DATA;
		}
	}

	NAV_CONST::NAV_MODE status = NAV_CONST::NAV_MODE::RAW_INERTIAL_DATA;
};

/**
 * @brief QLabel to hold information about accuracy
 */
class QDoubleLabel : public QStatusLabel {
	Q_OBJECT

   public:
	explicit QDoubleLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QInitializedLabel(parent, f);
	}
	virtual ~QDoubleLabel(){};

	/**
	 * @brief Set the value of the label and updates the style based on value
	 * @param acc Accuracy value
	 */
	void setValue(const double acc) {
		this->setText(QString::number(acc, 'f', 3));
		updateStyle(acc);
	}

	void setLimits(const double green, const double yellow, const double red = __DBL_MAX__) {
		limit.green = green;
		limit.yellow = yellow;
		limit.red = red;
	}

   private:
	/**
	 * @brief Update the style of the label based on the value
	 * @param acc Accuracy value
	 */
	void updateStyle(const double acc) {
		auto newStatus = evaluateStatus(acc);
		if (status == newStatus) {
			return;
		}

		status = newStatus;
		if (status == Status::OK) {
			this->setStyleSheet("background-color : green;");
		} else if (status == Status::WARNING) {
			this->setStyleSheet("background-color : yellow;");
		} else if (status == Status::ERROR) {
			this->setStyleSheet("background-color : red;");
		}
	}

	Status evaluateStatus(const double d) {
		if (d < limit.green) {
			return Status::OK;
		} else if (d >= limit.green && d < limit.yellow) {
			return Status::WARNING;
		} else {
			return Status::ERROR;
		}
	}

	DoubleLimit limit;
};
