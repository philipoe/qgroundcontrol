#ifndef _ASLUAV_H_
#define _ASLUAV_H_

#include "UAS.h"

class ASLUAV : public UAS
{
	Q_OBJECT
	Q_INTERFACES(UASInterface)

public:
	ASLUAV(void);
	ASLUAV(MAVLinkProtocol* mavlink, int id);
	~ASLUAV(void);

public:
    QString getAutopilotTypeName();	//Overwritten from UAS to add MAV_AUTOPILOT_ASLUAV type.

public slots:
    /** @brief Receive a MAVLink message from this MAV */
    void receiveMessage(LinkInterface* link, mavlink_message_t message);
	/** @brief Set Battery Specs for warnings, overwritten from UAS-class */
	void setBatterySpecs(const QString& specs);

signals:
	/** @brief CustomSensorData has changed */
	void AirspeedChanged(float airspeed);
	/** @brief CustomSensorData has changed */
    void AslctrlDataChanged(float uElev, float uAil, float uRud, float uThrot, float roll, float pitch, float yaw, float roll_ref, float pitch_ref, float h);

protected:
	// Add an external voltage sensor in addition to PX4-onboard sensor
	double currentVoltage_ext;	
	float lpVoltage_ext, tickLowpassVoltage_ext,lastTickVoltageValue_ext;
	float emptyVoltage_ext, warnVoltage_ext, fullVoltage_ext;
	float tickVoltage_ext;
	float startVoltage_ext;
};

#endif //_ASLUAV_H_
