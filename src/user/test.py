from user import User

test = User()
test.load_json("/home/hello-robot/matt.json")
print(test.name)

# me = User(name="Matt")
# me.save_json()