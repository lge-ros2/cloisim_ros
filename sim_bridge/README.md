# sim_bridge package

## Setup

It requires setting environment variable for simulation connection.

But if you not set environment, 127.0.0.1 shall be used for sim bridge ip in Code.

```shell
export CLOISIM_BRIDGE_IP='127.0.0.1'
```

## Constraints

Following bridges can be setup.
'publisher' and 'subscriber' can be setup simultaneously at one SimBridge module.

- 'publisher'
- 'subscriber
- 'publisher' + 'subscriber'
- 'service'
- 'client'

But, following bridge combinations can NOT be setup!

- 'publisher' + ('service' or 'client')
- 'subscriber' + ('service' or 'client')
- 'service' + 'client'
