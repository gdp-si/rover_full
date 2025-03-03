"""
This library is provided to allow standard python logging
to output log data as JSON formatted strings
"""
import importlib
import json
import logging
import re
import traceback
from collections import OrderedDict
from datetime import date, datetime, time, timezone
from inspect import istraceback

import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# skip natural LogRecord attributes
# http://docs.python.org/library/logging.html#logrecord-attributes
RESERVED_ATTRS = (
    "args",
    "asctime",
    "created",
    "exc_info",
    "exc_text",
    "filename",
    "funcName",
    "levelname",
    "levelno",
    "lineno",
    "module",
    "msecs",
    "message",
    "msg",
    "name",
    "pathname",
    "process",
    "processName",
    "relativeCreated",
    "stack_info",
    "thread",
    "threadName",
)


def merge_record_extra(record, target, reserved):
    """
    Merges extra attributes from LogRecord object into target dictionary

    Args:
        record : logging.LogRecord
        target : dict to update
        reserved : dict or list with reserved keys to skip
    """
    for key, value in record.__dict__.items():
        # this allows to have numeric keys
        if key not in reserved and not (
            hasattr(key, "startswith") and key.startswith("_")
        ):
            target[key] = value
    return target


class JsonEncoder(json.JSONEncoder):
    """
    A custom encoder extending the default JSONEncoder
    """

    def default(self, o):
        if isinstance(o, (date, datetime, time)):
            return self.format_datetime_obj(o)

        if istraceback(o):
            return "".join(traceback.format_tb(o)).strip()

        if (
            isinstance(o) == Exception
            or isinstance(o, Exception)
            or isinstance(o) == type
        ):
            return str(o)

        try:
            return super().default(o)

        except TypeError:
            try:
                return str(o)

            except Exception:
                return None

    @classmethod
    def format_datetime_obj(cls, obj):
        """Format DateTime object"""
        return obj.isoformat()


class JsonFormatter(logging.Formatter):
    """
    A custom formatter to format logging records as json strings.
    Extra values will be formatted as str() if not supported by
    json default encoder
    """

    def __init__(self, *args, **kwargs):
        """
        Initialize the formatter with a format string containing one or more
        Args:
            json_default: a function for encoding non-standard objects
                as outlined in http://docs.python.org/2/library/json.html
            json_encoder: optional custom encoder
            json_serializer: a :meth:`json.dumps`-compatible callable
                that will be used to serialize the log record.
            json_indent: an optional :meth:`json.dumps`-compatible numeric value
                that will be used to customize the indent of the output json.
            prefix: an optional string prefix added at the beginning of
                the formatted string
            rename_fields: an optional dict, used to rename field names in the output.
                Rename message to @message: {'message': '@message'}
            static_fields: an optional dict, used to add fields with static values to all logs
            json_indent: indent parameter for json.dumps
            json_ensure_ascii: ensure_ascii parameter for json.dumps
            reserved_attrs: an optional list of fields that will be skipped when
                outputting json log record. Defaults to all log record attributes:
                http://docs.python.org/library/logging.html#logrecord-attributes
            timestamp: an optional string/boolean field to add a timestamp when
                outputting the json log record. If string is passed, timestamp will be added
                to log record using string as key. If True boolean is passed, timestamp key
                will be "timestamp". Defaults to False/off.
        """
        self.json_default = self._str_to_fn(kwargs.pop("json_default", None))
        self.json_encoder = self._str_to_fn(kwargs.pop("json_encoder", None))
        self.json_serializer = self._str_to_fn(
            kwargs.pop("json_serializer", json.dumps)
        )
        self.json_indent = kwargs.pop("json_indent", None)
        self.json_ensure_ascii = kwargs.pop("json_ensure_ascii", True)
        self.prefix = kwargs.pop("prefix", "")
        self.rename_fields = kwargs.pop("rename_fields", {})
        self.static_fields = kwargs.pop("static_fields", {})
        reserved_attrs = kwargs.pop("reserved_attrs", RESERVED_ATTRS)
        self.reserved_attrs = dict(zip(reserved_attrs, reserved_attrs))
        self.timestamp = kwargs.pop("timestamp", False)

        # super(JsonFormatter, self).__init__(*args, **kwargs)
        logging.Formatter.__init__(self, *args, **kwargs)
        if not self.json_encoder and not self.json_default:
            self.json_encoder = JsonEncoder

        self._required_fields = self.parse()
        self._skip_fields = dict(zip(self._required_fields, self._required_fields))
        self._skip_fields.update(self.reserved_attrs)

    @classmethod
    def _str_to_fn(cls, fn_as_str):
        """
        If the argument is not a string, return whatever was passed in.
        Parses a string such as package.module.function, imports the module
        and returns the function.

        Args:
            fn_as_str - The string to parse. If not a string, return it.
        """
        if not isinstance(fn_as_str, str):
            return fn_as_str

        path, _, function = fn_as_str.rpartition(".")
        module = importlib.import_module(path)
        return getattr(module, function)

    def parse(self):
        """
        Parses format string looking for substitutions

        This method is responsible for returning a list of fields (as strings)
        to include in all log messages.
        """
        standard_formatters = re.compile(r"\((.+?)\)", re.IGNORECASE)
        return standard_formatters.findall(self._fmt)

    def add_fields(self, log_record, record, message_dict):
        """
        Override this method to implement custom logic for adding fields.
        """
        for field in self._required_fields:
            if field in self.rename_fields:
                log_record[self.rename_fields[field]] = record.__dict__.get(field)
            else:
                log_record[field] = record.__dict__.get(field)
        log_record.update(self.static_fields)
        log_record.update(message_dict)
        merge_record_extra(record, log_record, reserved=self._skip_fields)

        if self.timestamp:
            key = self.timestamp if isinstance(self.timestamp) == str else "timestamp"
            log_record[key] = datetime.fromtimestamp(record.created, tz=timezone.utc)

    @classmethod
    def process_log_record(cls, log_record):
        """
        Override this method to implement custom logic
        on the possibly ordered dictionary.
        """
        return log_record

    def jsonify_log_record(self, log_record):
        """Returns a json string of the log record."""
        return self.json_serializer(
            log_record,
            default=self.json_default,
            cls=self.json_encoder,
            indent=self.json_indent,
            ensure_ascii=self.json_ensure_ascii,
        )

    def serialize_log_record(self, log_record):
        """Returns the final representation of the log record."""
        return f"{self.prefix}{self.jsonify_log_record(log_record)}"

    def format(self, record):
        """Formats a log record and serializes to json"""
        message_dict = {}
        if isinstance(record.msg, dict):
            message_dict = record.msg
            record.message = None
        else:
            record.message = record.getMessage()
        # only format time if needed
        if "asctime" in self._required_fields:
            record.asctime = self.formatTime(record, self.datefmt)

        # Display formatted exception, but allow overriding it in the
        # user-supplied dict.
        if record.exc_info and not message_dict.get("exc_info"):
            message_dict["exc_info"] = self.formatException(record.exc_info)
        if not message_dict.get("exc_info") and record.exc_text:
            message_dict["exc_info"] = record.exc_text
        # Display formatted record of stack frames
        # default format is a string returned from :func:`traceback.print_stack`
        try:
            if record.stack_info and not message_dict.get("stack_info"):
                message_dict["stack_info"] = self.formatStack(record.stack_info)
        except AttributeError:
            # Python2.7 doesn't have stack_info.
            pass

        try:
            log_record = OrderedDict()
        except NameError:
            log_record = {}

        self.add_fields(log_record, record, message_dict)
        log_record = self.process_log_record(log_record)

        return self.serialize_log_record(log_record)


class HTTPHandler(logging.Handler):
    """HTTP Logging handler to stream over network"""

    def __init__(self, url: str, token: str, silent: bool = True):
        """
        Initializes the custom http handler
        Args:
            url (str): The URL that the logs will be sent to
            token (str): The Authorization token being used
            silent (bool): If False the http response and logs will be sent
                           to STDOUT for debug
        """
        self.url = url
        self.token = token
        self.silent = silent

        # sets up a session with the server
        self.MAX_POOLSIZE = 100
        self.session = session = requests.Session()
        session.headers.update(
            {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.token}",
            }
        )
        self.session.mount(
            "https://",
            HTTPAdapter(
                max_retries=Retry(
                    total=5, backoff_factor=0.5, status_forcelist=[403, 500]
                ),
                pool_connections=self.MAX_POOLSIZE,
                pool_maxsize=self.MAX_POOLSIZE,
            ),
        )

        super().__init__()

    def emit(self, record):
        """
        This function gets called when a log event gets emitted. It recieves a
        record, formats it and sends it to the url
        Args:
            record: a log record
        """
        logEntry = self.format(record)
        response = self.session.post(self.url, data=logEntry)

        if not self.silent:
            print(logEntry)
            print(response.content)


class Logger:
    """Logger for debugging and development"""

    def __init__(
        self,
        _module_name: str = "",
        _level=logging.DEBUG,
        _format: str = "[%(asctime)s] [%(name)s] [%(levelname)s] [%(message)s]",
        _datefmt: str = "%Y-%m-%d %H:%M:%S",
    ) -> None:
        self.logger = logging.getLogger(_module_name)
        self.logger.setLevel(_level)
        self.formatter = JsonFormatter(_format, _datefmt)

        # TODO: Add support for HTTPHandler
        # self.handler = HTTPHandler()
        self.handler = logging.StreamHandler()
        self.handler.setFormatter(self.formatter)
        self.logger.addHandler(self.handler)

    def INFO(self, _message: str) -> None:
        """Information that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.info(_message)

    def DEBUG(self, _message: str) -> None:
        """Debugging messages that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.debug(_message)

    def WARNING(self, _message: str) -> None:
        """Warning messages that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.warning(_message)

    def ERROR(self, _message: str, traceback: bool = False) -> None:
        """Error messages that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.error(_message, exc_info=traceback)

    def CRITICAL(self, _message: str, traceback: bool = False) -> None:
        """Critical error messages that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.critical(_message, exc_info=traceback)
