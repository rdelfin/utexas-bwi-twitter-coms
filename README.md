UT Austin BWI Twitter Communications
===================================

This project is part of the continuing building-wide intergration project of the Computer Science Department in The University of Texas at Austin. This provides a framework to communicate with the Twitter API and allow the robots to perform actions and have conversations with Twitter users.

Packages
-----

* **twitcher_manager**
  * **twitcher_manager_node**: Manager is in charge of forwarding messages and deciding when to respond to tweets.
* **twitcher_connection**
  * **twitcher_connection_node**: This node is in charge of receiving and sending Tweets. It directly connects with the Twitter API.
* **twitcher_interpreter**
  * **twitcher_interpreter_node**: This node converts a given node into an action message, such as going to a certain room or finding a person.
* **twitcher_actions**
  * **twitcher_actions_node**: This node executes the actions send by the interpreter/manager node. It does this through actionlib servers.
* **twitcher_launch**
  * **twitcher.launch**: Launches all the above nodes in the correct order. While, order does not necesarily matter, it is better to execute nodes through this file.
