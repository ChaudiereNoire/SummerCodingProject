package org.phs.code.motors;

public class ReverseMotor extends Motor {

	private final Motor delegatingMotor;

	public ReverseMotor(Motor delegatingMotor) {
		this.delegatingMotor = delegatingMotor;
	}

	@Override
	public void setSpeed(int speed) {
		delegatingMotor.setSpeed( -speed);
	}

}