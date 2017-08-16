var arDrone = require('ar-drone');
var client  = arDrone.createClient();

var PubSub = function ()
{
    var handlers = {};
 
    this.subscribe = function (event, handler)
    {
        if (handlers[event] === undefined)  handlers[event] = [];
        handlers[event].push(handler);
    };
 
    /*this.publish = function (event)
    {
        if (handlers[event] === undefined) return;
 
        var i = 0,
            len = handlers[event].length;
 
        for (i; i < len; i++)
        {
            handlers[event][i](arguments[i+1]);
        }
    };*/
};
 
pubSub = new PubSub();
pubSub.subscribe('command', function(arg){
                                                alert("myEvent worked. Arg: " + typeof(arg));
                                                /*switch() {
                                                        case '1':
                                                                client.right();
                                                                break;
                                                        case '2':
                                                                client.left();
                                                                break;
                                                        case '3':
                                                                client.front();
                                                                break;
                                                        case '4':
                                                                client.back();
                                                                break;
                                                }*/

                                          });


