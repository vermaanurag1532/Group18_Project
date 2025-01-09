from rasa_sdk import Action
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet
from typing import Any, Text, Dict, List
import requests

BASE_URL = "http://localhost:3000"

# Helper function to handle HTTP requests
def fetch_data(endpoint: str, method: str = "GET", payload: Dict = None) -> Dict:
    try:
        if method == "GET":
            response = requests.get(f"{BASE_URL}/{endpoint}")
        elif method == "POST":
            response = requests.post(f"{BASE_URL}/{endpoint}", json=payload)
        else:
            return {"error": "Invalid HTTP method"}

        if response.status_code in [200, 201]:
            return response.json()
        else:
            return {"error": f"Failed with status code {response.status_code}"}
    except requests.exceptions.RequestException as e:
        return {"error": str(e)}
    
class ActionHandleSuggestions(Action):
    def name(self) -> Text:
        return "action_handle_suggestions"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Any,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        dispatcher.utter_message(text="Here are some suggestions for you!")
        return []


class ActionFetchMenu(Action):
    def name(self) -> Text:
        return "action_fetch_menu"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Any,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        menu_data = fetch_data("Menu")
        if "error" in menu_data:
            dispatcher.utter_message(text="Sorry, I'm unable to fetch the menu at the moment.")
        else:
            menu_message = "Here is our menu:\n"
            for item in menu_data:
                menu_message += f"- {item['Dish Name']} ({item['discription']}): ${item['Price']}\n"
            dispatcher.utter_message(text=menu_message)
        
        return []


class ActionPlaceOrder(Action):
    def name(self) -> Text:
        return "action_place_order"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Any,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        item_name = tracker.get_slot("item_name")
        quantity = tracker.get_slot("quantity")
        customer_name = tracker.get_slot("customer_name")

        if not item_name or not quantity or not customer_name:
            dispatcher.utter_message(
                text="It seems some order details are missing. Please provide the item name, quantity, and your name."
            )
            return []

        # Check or create customer
        customer_response = fetch_data(f"Customer?name={customer_name}")
        if "error" not in customer_response and customer_response:
            customer_id = customer_response[0]["Customer Id"]
        else:
            new_customer_data = {"Customer Name": customer_name}
            new_customer_response = fetch_data("Customer", method="POST", payload=new_customer_data)
            if "error" in new_customer_response:
                dispatcher.utter_message(text="Failed to register your details. Please try again.")
                return []
            customer_id = new_customer_response["Customer Id"]

        # Calculate order total (fetch item price dynamically if available)
        menu_data = fetch_data("Menu")
        if "error" in menu_data:
            dispatcher.utter_message(text="Unable to calculate the total. Please try again.")
            return []
        
        item_price = next((item["Price"] for item in menu_data if item["Dish Name"].lower() == item_name.lower()), None)
        if not item_price:
            dispatcher.utter_message(text=f"Sorry, {item_name} is not available on the menu.")
            return []

        order_total = float(quantity) * float(item_price)

        # Place the order
        order_data = {
            "Customer Id": customer_id,
            "Order Description": f"{quantity} x {item_name}",
            "Total": order_total
        }
        order_response = fetch_data("Order", method="POST", payload=order_data)

        if "error" in order_response:
            dispatcher.utter_message(text="Failed to place your order. Please try again.")
        else:
            dispatcher.utter_message(
                text=f"Thank you, {customer_name}! Your order for {quantity} {item_name} has been placed successfully. Your total is ${order_total}."
            )
            dispatcher.utter_message(text="Would you like to confirm the payment now?")

        # Reset slots
        return [SlotSet("item_name", None), SlotSet("quantity", None), SlotSet("customer_name", None)]
