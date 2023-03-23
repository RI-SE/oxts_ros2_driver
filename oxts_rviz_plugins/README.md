# OxTs RViz plugins

## Status Panel
RVIZ plugin built together with the rest of the _oxts_ package for showing oxts status as a panel.
Is added in rviz by clicking "Panels" -> "Add New Panel". Then select the _oxts_rviz_plugins/StatusPanel_

![Selection](https://user-images.githubusercontent.com/23191287/225628456-5ae381c8-f326-48b8-895a-b352d24657b9.png)

Run with arbitrary NCOM file or unit and output should look like this:

![statuspanel](https://user-images.githubusercontent.com/23191287/225628643-915130c3-886a-4880-a4f3-a6822308f693.png)


Edit the "NCOM Topic" to match the output topic of _oxts_driver_ package. Defaults to _/ins/ncom_.