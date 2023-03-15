#pragma once

#include <QLabel>
#include <QWidget>

#include "oxts_rviz_plugins/nav_const.hpp"

enum class Status { OK, WARNING, ERROR };

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

class QTextLabel : public QLabel {
	Q_OBJECT
   public:
	explicit QTextLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QLabel(parent, f);
		this->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
		this->setWordWrap(true);
	}

	explicit QTextLabel(QWidget* parent, QString text, Qt::WindowFlags f = Qt::WindowFlags()) {
		QLabel(parent, f);
		this->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
		this->setWordWrap(true);
		this->setText(text);
	}

	virtual ~QTextLabel(){};
};

class QSatStatusLabel : public QLabel {
	Q_OBJECT

   public:
	explicit QSatStatusLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QLabel(parent, f);
		this->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
		this->setWordWrap(true);
		this->setText("Awaiting data");
		this->setStyleSheet("QLabel { background-color : red; color : black; }");
	}
	virtual ~QSatStatusLabel(){};

	void setValue(const int nSats) {
		this->setText(QString::number(nSats));
		this->updateStyle(nSats);
	}

   private:
	void updateStyle(const int text) {
		auto newStatus = getStatus(text);
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

	Status getStatus(const int d) {
		if (d > limit.green) {
			return Status::OK;
		} else if (d >= limit.yellow && d < limit.green) {
			return Status::WARNING;
		} else {
			return Status::ERROR;
		}
	}

	Status status = Status::ERROR;
	GpsLimits limit;
};

class QNavStatusLabel : public QLabel {
	Q_OBJECT
   public:
	explicit QNavStatusLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QLabel(parent, f);
		this->setText("Awaiting data");
		this->setStyleSheet("QLabel { background-color : red; color : black; }");
	}
	virtual ~QNavStatusLabel(){};

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

class QDoubleLabel : public QLabel {
	Q_OBJECT

   public:
	explicit QDoubleLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
		QLabel(parent, f);
		this->setText("Awaiting data");
		this->setStyleSheet("QLabel { background-color : red; color : black; }");
	}
	virtual ~QDoubleLabel(){};

	void setValue(const double d_text) {
		this->setText(QString::number(d_text, 'f', 3));
		updateStyle(d_text);
	}

    void setLimits(const double green, const double yellow, const double red = __DBL_MAX__) {
        limit.green = green;
        limit.yellow = yellow;
        limit.red = red;
    }

   private:
	void updateStyle(const double d_text) {
		auto newStatus = getStatus(d_text);
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

	Status getStatus(const double d) {
		if (d < limit.green) {
			return Status::OK;
		} else if (d >= limit.green && d < limit.yellow) {
			return Status::WARNING;
		} else {
			return Status::ERROR;
		}
	}

	Status status = Status::ERROR;
	DoubleLimit limit;
};

