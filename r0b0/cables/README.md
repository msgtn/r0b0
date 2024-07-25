
# Cables
Cables (`r0b0.cables`) are the input-output translators for events.
They translate information from an input event into instructions for an output event.
They accept and return `dict`s in the form of [`socketio` events sent through `emit`s](https://python-socketio.readthedocs.io/en/latest/client.html#emitting-events).


To connect `Gadget`s through a `Cable`, define the message function in any file in `r0b0/cables`, the input/transmitting `Gadget` (`tx_gadget`), and the output/receiving `Gadget` (`rx_gadget`) in the `Rig`'s `config.yaml`:
```
gadgets:
- input_gadget
- output_gadget
cables:
- cable: input2output
  tx_gadget: input_gadget
  rx_gadget: output_gadget
```

The basic skeleton for the `input2output` `Cable` function is:

```
def input2output(data=None):
  if data is None:
		return {'event':'input_event'}

  output_data = translation_function_that_outputs_a_dictionary(data)

	return {
		'event':'output_event',
    'data':output_data
		# add more arguments as needed
	}
```

In the above example, the Rig will call the `input2output` `Cable` function when it receives an `input_event` event emitted from the `input_gadget`.
The `Cable` outputs the `data` dictionary to the `Rig`, which emits the `output_event` and `output_data` payload to the `output_gadget`.

Alternatively, one could combine the incoming data with the translated output data by using a dictionary update instead of defining a new dictionary:
```
...
  data.update(translation_function_that_outputs_a_dictionary(data))

  return {
    'event':output_event,
    'data':data
  }
...
```

## Creating new Cables

### Cable management

The [`__init__.py`](./__init__.py) file will pull all modules and functions from all subfiles of the `r0b0/cables` directory, so you can keep functions separated by application.
For example, you could keep functions relevant to many robots in a `robot_cables.py` file, and Rig-specific functions in a `cables_for_a_specific_rig.py` file.
The import order is sorted alphabetically by filename, so it's good practice to give each Cable function a unique name.

### Cable definition

*The boilerplate is a kludge; I'm thinking of ways to use a decorator or something to simplify the function definition.*

Let's look again at the example function from above:
```
def input2output(data=None):
  if data is None:
		return {'event':'input_event'}

  output_data = translation_function_that_outputs_a_dictionary(data)

	return {
		'event':'output_event',
    'data':output_data
		# add more arguments as needed
	}
```

This is the basic skeleton for any Cable function.
The function takes a `data` input argument.
When the Rig connect a Cable function, it will call the function with no argument (`data=None`) to get the `input_event`.
*(This could be moved to the Rig's config yaml, but I like having it more closely coupled to the Cable for legibility of the `input_event`. Defining the `input_event` in the config will complicate the yaml and require more knowledge of the Gadget innards from the user's perspective.)*
The Rig will only call this function when `input_event` is emitted on the `input_gadget`'s namespace.
`translation_function_that_outputs_a_dictionary()` is a stand-in for how the input data will define the output data.
The function outputs a dictionary with the `output_event` and `data` -- these will serve as the `kwargs` for the Rig to emit the `output_event` to the `output_gadget`'s namespace.
