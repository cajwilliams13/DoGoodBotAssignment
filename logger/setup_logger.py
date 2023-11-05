import json
import logging


class JSONFormatter(logging.Formatter):
    def __init__(self) -> None:
        super().__init__()

        self._ignore_keys = {"msg", "args"}

    def format(self, record: logging.LogRecord) -> str:
        message = record.__dict__.copy()
        message["message"] = record.getMessage()

        for key in self._ignore_keys:
            message.pop(key, None)

        if record.exc_info and record.exc_text is None:
            record.exc_text = self.formatException(record.exc_info)

        if record.exc_text:
            message["exc_info"] = record.exc_text

        if record.stack_info:
            message["stack_info"] = self.formatStack(record.stack_info)

        return json.dumps(message)

def setup_logger():
    logger = logging.getLogger() # Get root logger
    logger.setLevel(logging.DEBUG)

    # handler = logging.StreamHandler()
    # handler.setFormatter(JSONFormatter())

    # logger.addHandler(handler)

    logger.debug("Logger has been setup")
