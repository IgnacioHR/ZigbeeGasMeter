const m = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['GasMeter'],
    model: 'GasMeter',
    vendor: 'MICASA',
    description: 'Zigbee Gas meter created by Ignacio Hernández-Ros',
    ota: true,
    extend: [/*m.identify({"isSleepy":true}),*/ 
      m.gasMeter({
        cluster:"metering", 
        producedEnergy: false
      }), 
      m.battery({
        voltage: true, 
        lowStatus: true
      }),
      m.numeric({
        name: "gas_counter", 
        unit: "m³", 
        precision: 2, 
        cluster: "genAnalogInput", 
        attribute: "presentValue", 
        description: "Alternate way to obtain the gas consumption value", 
        access: "STATE_GET", 
        reporting: {min: 10, max: 65000, change: 10}
      })
    ],
    meta: {},
    configure: async (device, coordinatorEndpoint) => {
      await device.getEndpoint(1).read("genAnalogInput", ["presentValue"])
    },
};

module.exports = definition;