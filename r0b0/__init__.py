import logging as logging
logging.basicConfig(
    encoding='utf-8',
    # level=logging.DEBUG,
    level=logging.WARNING,
    )

import time
get_timestamp = lambda: time.strftime('%Y%m%d%H%M%S')

