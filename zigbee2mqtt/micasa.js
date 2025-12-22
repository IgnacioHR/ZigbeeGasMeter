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
      }), 
      m.battery({
        voltage: true, 
        lowStatus: true
      }),
    ],
    meta: {},
};

module.exports = definition;