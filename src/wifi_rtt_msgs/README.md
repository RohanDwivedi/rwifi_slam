# wifi_rtt_msgs

Message definitions for Wi-Fi RTT range observations.

## `APRangeObservation`

One scan: every access point measured at the same time.

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Scan time; `frame_id` is the antenna frame |
| `observations` | `APRange[]` | One entry per access point |

## `APRange`

Range to a single access point. All distances are in metres.

| Field | Type | Description |
|---|---|---|
| `mac_address` | `string` | Access point MAC address |
| `measured_distance` | `float32` | Range reported by the sensor; the only field a SLAM consumer should use |
| `true_distance` | `float32` | Ground-truth distance (simulation only) |
| `nlos_bias` | `float32` | Non-line-of-sight bias included in `measured_distance` (simulation only) |
| `measurement_noise` | `float32` | Noise sample included in `measured_distance` (simulation only) |
| `hardware_bias` | `float32` | Per-access-point device bias (simulation only) |
| `los` | `bool` | Whether the line of sight was clear (simulation only) |
