const ERROR_KANGAROO= 1;
const ERROR_SPI     = 2;
const ERROR_BRAIN   = 4;
const ERROR_TELEOP  = 8;

class Dashboard{
    constructor(){
        this.socket = io.connect(window.location.origin);
        this.socket.on('gps',this.onGps.bind(this));
        this.socket.on('exception',this.onException.bind(this));
        this.socket.on('nodebeat', this.onBeat.bind(this));

        this.gpsDgnst = true;
        this.controlDgnst = true;

        console.log('Vehiculo conectado.');
    }

    onGps(data){
        if(this.gpsDgnst){
            for (var key in data){
                var selector = "#" + key;
                if(data[key]){
                    $(selector).css({color: "green"});
                } else{
                    $(selector).css({color: "gray"});
                }
            }
        }
    }

    onException(data){
        if(this.controlDgnst){
            if (data["data"] & ERROR_KANGAROO){
                $("#kangaroo-error").css({color: "red"});
            } else {
                $("#kangaroo-error").css({color: "gray"});
            }
            if (data["data"] & ERROR_SPI){
                $("#spi-error").css({color: "red"});
            } else {
                $("#spi-error").css({color: "gray"});
            }
            if (data["data"] & ERROR_BRAIN){
                $("#brain-error").css({color: "red"});
            } else {
                $("#brain-error").css({color: "gray"});
            }
            if (data["data"] & ERROR_TELEOP){
                $("#teleop-error").css({color: "red"});
            } else {
                $("#teleop-error").css({color: "gray"});
            }
        }
    }

    onBeat(data){
        
    }
}

const dashboard = new Dashboard();

$(function(){
    
    $("#content-header").html("Overview");
    $.getJSON("./icons-diagnosis.json", function (json) {
        $.each(json, function (i, item) {
            $('<tr>').append('<th colspan="5" class="table-subheader">' + i + '</th>').appendTo("#icons_table");
            var row = $('<tr>');
            $.each(item, (j, subitem) => {
                var cell = $('<td>');
                cell.append('<i id="' + subitem.id + '" class="fad ' + subitem.icon +' fa-2x"></i>');
                cell.hover(
                    function(){$(this).attr('title',subitem.description)},
                    function(){}
                );
                row.append(cell);
            });
            row.appendTo('#icons_table');
            row.wrap('<p>').html();
        });
    });

    $.getJSON("./icons-nodes.json",function(json) {
        $.each(json, function(i,item){
            $('<tr>').append('<th colspan="5" class="table-subheader">'+i+'</th>').appendTo("#nodes_table");
            var icons = 0;
            var row = $('<tr>');
            $.each(item, (i,subitem) => {
                icons += 1;
                var cell = $('<td>');
                cell.append('<i id="' + subitem.id + '" class="fad ' + subitem.icon +' fa-2x"></i>');
                cell.hover(
                    function(){$(this).attr('title',subitem.description)},
                    function(){}
                );
                row.append(cell);
                if(icons == 5)
                {
                    row.appendTo('#nodes_table');
                    row = $('<tr>');
                    icons = 0;
                }
            });
            row.appendTo('#nodes_table');
        });
    });
});