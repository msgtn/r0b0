import logging


def ps2014app(data=None):
    if data is None:
        return {"event": "button"}

    motor_id = [1]
    value = [data["value"]]

    logging.debug(f"button to position event {value}")

    return {"event": "position", "motor_id": motor_id, "value": value}
