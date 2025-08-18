class NodeAttribute:
    TAG_MODE1 = "mode1"
    TAG_MODE2 = "mode2"
    TAG_PARK = "park"
    TAG_TRAFFIC_LIGHTS_ID = "traffic_lights_id"
    # TAG_TRAFFIC_LIGHTS_DIR = "traffic_lights_dir"

    TAG_NAMES = [
        TAG_MODE1,
        TAG_MODE2,
        TAG_PARK,
        TAG_TRAFFIC_LIGHTS_ID,
        # TAG_TRAFFIC_LIGHTS_DIR,
    ]

    def __init__(self):
        self.tags = {}

    def add_tag(self, key, value):
        self.tags[key] = value

    def get_tag_value(self, key, default=None):
        if key in self.tags:
            return self.tags[key]
        return default

    def __str__(self):
        return str(self.tags)

    def __repr__(self):
        return str(self.tags)
