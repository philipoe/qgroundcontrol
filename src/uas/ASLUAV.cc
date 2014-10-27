#include "ASLUAV.h"
#include <iostream>
#include <QString>
#include "GAudioOutput.h"

ASLUAV::ASLUAV(MAVLinkProtocol* mavlink, QThread* thread, int id)
	: UAS(mavlink, thread, id), 
	emptyVoltage1(9.0), 
	warnVoltage1(10.5),
	fullVoltage1(12.5),
	tickVoltage1(warnVoltage1),
	lpVoltage1(0.0),
	tickLowpassVoltage1(0.0),
	lastTickVoltageValue1(0.0)
{
}


ASLUAV::~ASLUAV(void)
{
}

//Overwritten function is in UAS
QString ASLUAV::getAutopilotTypeName()
{
    switch (autopilot)
    {
    case MAV_AUTOPILOT_ASLUAV:
        return "ASLUAV";
        break;
    default:
		return UAS::getAutopilotTypeName();
        break;
    }
}



void ASLUAV::receiveMessage(LinkInterface *link, mavlink_message_t message)
{
#ifdef MAVLINK_ENABLED_ASLUAV
	//std::cout << "msgid:"<<(int)message.msgid<<endl;

	if (!link) return;
    if (!links->contains(link))
    {
        addLink(link);
        //        qDebug() << __FILE__ << __LINE__ << "ADDED LINK!" << link->getName();
    }

    // Only accept messages from this system (condition 1)
    // and only then if a) attitudeStamped is disabled OR b) attitudeStamped is enabled
    // and we already got one attitude packet
    if (message.sysid == uasId && (!attitudeStamped || (attitudeStamped && (lastAttitude != 0)) || message.msgid == MAVLINK_MSG_ID_ATTITUDE))
    {
        QString uasState;
        QString stateDescription;

        bool multiComponentSourceDetected = false;
        bool wrongComponent = false;

        switch (message.compid)
        {
        case MAV_COMP_ID_IMU_2:
            // Prefer IMU 2 over IMU 1 (FIXME)
            componentID[message.msgid] = MAV_COMP_ID_IMU_2;
            break;
        default:
            // Do nothing
            break;
        }

        // Store component ID
        if (componentID[message.msgid] == -1)
        {
            // Prefer the first component
            componentID[message.msgid] = message.compid;
        }
        else
        {
            // Got this message already
            if (componentID[message.msgid] != message.compid)
            {
                componentMulti[message.msgid] = true;
                wrongComponent = true;
            }
        }

        if (componentMulti[message.msgid] == true) multiComponentSourceDetected = true;


        //******************************************************************************
		//*** SWITCH CUSTOM ASLUAV-MESSAGES HERE
		//******************************************************************************
		switch (message.msgid)
        {
            case MAVLINK_MSG_ID_SENS_POWER:
			{
                //std::cout << "ASLUAV: Received SENS_POWER data message"<<endl;
				
                mavlink_sens_power_t data;
                mavlink_msg_sens_power_decode(&message, &data);

				// Battery charge/time remaining/voltage calculations
				currentVoltage1 = data.adc121_vspb_volt;
				lpVoltage1 = lpVoltage1*0.7f+0.3*currentVoltage1; //Was: UAS::filterVoltage()
				tickLowpassVoltage1 = tickLowpassVoltage1*0.8f + 0.2f*currentVoltage1;

				// We don't want to tick above the threshold
				if (tickLowpassVoltage1 > tickVoltage1)
				{
					lastTickVoltageValue1 = tickLowpassVoltage1;
				}

				if ((startVoltage > 0.0f) && (tickLowpassVoltage1 < tickVoltage1) && (fabs(lastTickVoltageValue1 - tickLowpassVoltage1) > 0.1f)
						/* warn if lower than treshold */
						&& (lpVoltage1 < tickVoltage1)
						/* warn only if we have at least the voltage of an empty LiPo cell, else we're sampling something wrong */
						&& (currentVoltage1 > 3.3f)
						/* warn only if current voltage is really still lower by a reasonable amount */
						&& ((currentVoltage1 - 0.2f) < tickVoltage1)
						/* warn only every 12 seconds */
						&& (QGC::groundTimeUsecs() - lastVoltageWarning) > 12000000)
				{
					GAudioOutput::instance()->say(QString("ADC121 voltage warning: %1 volts").arg(lpVoltage1, 0, 'f', 1, QChar(' ')));
					lastVoltageWarning = QGC::groundTimeUsecs();
					lastTickVoltageValue1 = tickLowpassVoltage1;
				}

				//The rest of the message handling (i.e. adding data to the plots) is done elsewhere
				break;
			}
			case MAVLINK_MSG_ID_VFR_HUD:
			{
				// Emit custom_sensor_data updated signal. This is primarily for the autotune-widget
				// Only send airspeed for now, because only this is needed by autotune-widget.
				mavlink_vfr_hud_t data;	//TODO Note: It might be better to get this from a separate airspeed-mavlink-message, however that does not exist yet.
				mavlink_msg_vfr_hud_decode(&message, &data);
				emit AirspeedChanged(data.airspeed);
				break;
			}
			case MAVLINK_MSG_ID_SYS_STATUS: 
			// overwritten, normally handled in PxQuadMAV::receiveMessage() or UAS::receiveMessage()
			// mods introduced apply to voltage calculations & alarms (PhOe)
			{
				if (multiComponentSourceDetected && wrongComponent)
				{
					break;
				}
				mavlink_sys_status_t state;
				mavlink_msg_sys_status_decode(&message, &state);

				// Prepare for sending data to the realtime plotter, which is every field excluding onboard_control_sensors_present.
				quint64 time = getUnixTime();
				QString name = QString("M%1:SYS_STATUS.%2").arg(message.sysid);
				emit valueChanged(uasId, name.arg("sensors_enabled"), "bits", state.onboard_control_sensors_enabled, time);
				emit valueChanged(uasId, name.arg("sensors_health"), "bits", state.onboard_control_sensors_health, time);
				emit valueChanged(uasId, name.arg("errors_comm"), "-", state.errors_comm, time);
				emit valueChanged(uasId, name.arg("errors_count1"), "-", state.errors_count1, time);
				emit valueChanged(uasId, name.arg("errors_count2"), "-", state.errors_count2, time);
				emit valueChanged(uasId, name.arg("errors_count3"), "-", state.errors_count3, time);
				emit valueChanged(uasId, name.arg("errors_count4"), "-", state.errors_count4, time);

				// Process CPU load.
				emit loadChanged(this,state.load/10.0f);
				emit valueChanged(uasId, name.arg("load"), "%", state.load/10.0f, time);

				// Battery charge/time remaining/voltage calculations
				currentVoltage = state.voltage_battery/1000.0f;
				lpVoltage = filterVoltage(currentVoltage);
				tickLowpassVoltage = tickLowpassVoltage*0.8f + 0.2f*currentVoltage;

				// We don't want to tick above the threshold
				if (tickLowpassVoltage > tickVoltage)
				{
					lastTickVoltageValue = tickLowpassVoltage;
				}

				if ((startVoltage > 0.0f) && (tickLowpassVoltage < tickVoltage) && (fabs(lastTickVoltageValue - tickLowpassVoltage) > 0.1f)
						/* warn if lower than treshold */
						&& (lpVoltage < tickVoltage)
						/* warn only if we have at least the voltage of an empty LiPo cell, else we're sampling something wrong */
						&& (currentVoltage > 3.3f)
						/* warn only if current voltage is really still lower by a reasonable amount */
						&& ((currentVoltage - 0.2f) < tickVoltage)
						/* warn only every 12 seconds */
						&& (QGC::groundTimeUsecs() - lastVoltageWarning) > 12000000)
				{
					GAudioOutput::instance()->say(QString("voltage warning: %1 volts").arg(lpVoltage, 0, 'f', 1, QChar(' ')));
					lastVoltageWarning = QGC::groundTimeUsecs();
					lastTickVoltageValue = tickLowpassVoltage;
				}

				if (startVoltage == -1.0f && currentVoltage > 0.1f) startVoltage = currentVoltage;
				timeRemaining = calculateTimeRemaining();
				if (!batteryRemainingEstimateEnabled && chargeLevel != -1)
				{
					chargeLevel = state.battery_remaining;
				}

				emit batteryChanged(this, lpVoltage, currentCurrent, getChargeLevel(), timeRemaining);
				emit valueChanged(uasId, name.arg("battery_remaining"), "%", getChargeLevel(), time);
				// emit voltageChanged(message.sysid, currentVoltage);
				emit valueChanged(uasId, name.arg("battery_voltage"), "V", currentVoltage, time);

				// And if the battery current draw is measured, log that also.
				if (state.current_battery != -1)
				{
					currentCurrent = ((double)state.current_battery)/100.0f;
					emit valueChanged(uasId, name.arg("battery_current"), "A", currentCurrent, time);
				}

				// LOW BATTERY ALARM
				if (lpVoltage < warnVoltage && (currentVoltage - 0.2f) < warnVoltage && (currentVoltage > 3.3))
				{
					// An audio alarm. Does not generate any signals.
					startLowBattAlarm();
				}
				else
				{
					stopLowBattAlarm();
				}

				// control_sensors_enabled:
				// relevant bits: 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control
				emit attitudeControlEnabled(state.onboard_control_sensors_enabled & (1 << 11));
				emit positionYawControlEnabled(state.onboard_control_sensors_enabled & (1 << 12));
				emit positionZControlEnabled(state.onboard_control_sensors_enabled & (1 << 13));
				emit positionXYControlEnabled(state.onboard_control_sensors_enabled & (1 << 14));

				// Trigger drop rate updates as needed. Here we convert the incoming
				// drop_rate_comm value from 1/100 of a percent in a uint16 to a true
				// percentage as a float. We also cap the incoming value at 100% as defined
				// by the MAVLink specifications.
				if (state.drop_rate_comm > 10000)
				{
					state.drop_rate_comm = 10000;
				}
				emit dropRateChanged(this->getUASID(), state.drop_rate_comm/100.0f);
				emit valueChanged(uasId, name.arg("drop_rate_comm"), "%", state.drop_rate_comm/100.0f, time);
			
				break;
			}
			case MAVLINK_MSG_ID_ASLCTRL_DATA:
			{
				mavlink_aslctrl_data_t data;
				mavlink_msg_aslctrl_data_decode(&message, &data);
								
				emit AslctrlDataChanged(data.uElev, data.uAil, data.uRud, data.uThrot, data.RollAngle,
					data.PitchAngle, data.YawAngle, data.RollAngleRef, data.PitchAngleRef, data.h);
				
				//receiveMessage(LinkInterface *link, mavlink_message_t message)
				
				//The rest of the message handling (i.e. adding data to the plots) is done elsewhere
				break;
			}
			default:
			{
				//std::cout << "msgid:"<<message.(int)msgid<<endl; //"ASLUAV: Unknown message received"<<endl;
                UAS::receiveMessage(link, message);
				//Q_UNUSED(link);
				//Q_UNUSED(message);
				break;
			}
		}	
	}

#else
    // Let UAS handle the default message set
    UAS::receiveMessage(link, message);
    Q_UNUSED(link);
    Q_UNUSED(message);
#endif // MAVLINK_ENABLED_ASLUAV
}

void ASLUAV::setBatterySpecs(const QString& specs)
{
    if (specs.length() == 0 || specs.contains("%"))
    {
        //TODO: Check why QGC sometimes crashes when using the setBatterySpecs function for adc121_XXX

        batteryRemainingEstimateEnabled = false;
        bool ok;
        QString percent = specs;
        percent = percent.remove("%");
        float temp = percent.toFloat(&ok);
        if (ok)
        {
            warnLevelPercent = temp;
        }
        else
        {
            emit textMessageReceived(0, 0, 0, "Could not set battery options, format is wrong");
        }
    }
    else
    {
        batteryRemainingEstimateEnabled = false;
        QString stringList = specs;
        stringList = stringList.remove("V");
        stringList = stringList.remove("v");
        QStringList parts = stringList.split(",");
        if (parts.length() == 3)
        {
            float temp;
            bool ok;
            // Get the empty voltage
            temp = parts.at(0).toFloat(&ok);
            if (ok) emptyVoltage = temp;
            // Get the warning voltage
            temp = parts.at(1).toFloat(&ok);
            if (ok) warnVoltage = temp;
            // Get the full voltage
            temp = parts.at(2).toFloat(&ok);
            if (ok) fullVoltage = temp;

			tickVoltage=warnVoltage;
        }
		else if (parts.length() == 6)
        {
            float temp;
            bool ok;
            // Get the empty voltage
            temp = parts.at(0).toFloat(&ok);
            if (ok) emptyVoltage = temp;
            // Get the warning voltage
            temp = parts.at(1).toFloat(&ok);
            if (ok) warnVoltage = temp;
            // Get the full voltage
            temp = parts.at(2).toFloat(&ok);
            if (ok) fullVoltage = temp;
			// Get the empty voltage
            temp = parts.at(3).toFloat(&ok);
            if (ok) emptyVoltage1 = temp;
            // Get the warning voltage
            temp = parts.at(4).toFloat(&ok);
            if (ok) warnVoltage1 = temp;
            // Get the full voltage
            temp = parts.at(5).toFloat(&ok);
            if (ok) fullVoltage1 = temp;

			tickVoltage=warnVoltage;
			tickVoltage1=warnVoltage1;
        }
        else
        {
            emit textMessageReceived(0, 0, 0, "Could not set battery options, format is wrong");
        }
    }
}
